#!/usr/bin/env bash
set -euo pipefail

die() { echo "error: $*" >&2; exit 1; }
info() { echo "info: $*" >&2; }

NEVERSSL_URL="${NEVERSSL_URL:-http://neverssl.com}"
TEST_HTTPS_URL="${TEST_HTTPS_URL:-https://example.com}"
PING_IP="${PING_IP:-1.1.1.1}"
SLEEP_SECS="${SLEEP_SECS:-5}"

need_cmd() { command -v "$1" >/dev/null 2>&1 || die "missing required command: $1"; }

has_internet() {
	# Prefer HTTPS test; some networks block ICMP.
	curl -fsSIL --max-time 6 "$TEST_HTTPS_URL" >/dev/null 2>&1
}

default_route_dev() {
	ip -4 route get "$PING_IP" 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}'
}

prefer_wifi_routes_runtime() {
	# If a USB gadget bridge default route exists, it often steals default traffic.
	# Delete only the specific l4tbr0 default routes (runtime change).
	local changed=0
	while IFS= read -r line; do
		# example: default via 192.168.55.100 dev l4tbr0 metric 50
		if [[ "$line" =~ ^default\ via\ ([0-9.]+)\ dev\ (l4tbr0)\ metric\ ([0-9]+) ]]; then
			local via="${BASH_REMATCH[1]}"
			local dev="${BASH_REMATCH[2]}"
			local metric="${BASH_REMATCH[3]}"
			ip route del default via "$via" dev "$dev" metric "$metric" >/dev/null 2>&1 || true
			changed=1
		fi
	done < <(ip -4 route show default || true)
	[[ "$changed" -eq 0 ]] || info "removed l4tbr0 default route(s) for this session"
}

print_wifi_list() {
	info "scanning Wi‑Fi..."
	# Columns: SSID, SECURITY, SIGNAL. -t gives ':' delimited, easier to parse.
	nmcli -t -f SSID,SECURITY,SIGNAL dev wifi list | awk -F: '
		BEGIN { print ""; printf "%-4s  %-35s  %-18s  %s\n", "No.", "SSID", "SECURITY", "SIGNAL"; print "----  -----------------------------------  ------------------  ------" }
		{
			ssid=$1; sec=$2; sig=$3;
			gsub(/^[ \t]+|[ \t]+$/, "", ssid);
			if (ssid == "") next;
			key=ssid "|" sec;
			if (seen[key]++) next;
			n++;
			printf "%-4d  %-35s  %-18s  %s\n", n, ssid, (sec==""?"--":sec), sig
			ssids[n]=ssid; secs[n]=sec
		}
		END { print "" }
	'
}

pick_wifi_from_nmcli() {
	# Emits: "SSID|SECURITY"
	local list
	list="$(nmcli -t -f SSID,SECURITY dev wifi list | awk -F: '
		{
			ssid=$1; sec=$2;
			gsub(/^[ \t]+|[ \t]+$/, "", ssid);
			if (ssid == "") next;
			key=ssid "|" sec;
			if (seen[key]++) next;
			n++;
			ssids[n]=ssid; secs[n]=sec;
		}
		END {
			for (i=1;i<=n;i++) printf "%s|%s\n", ssids[i], secs[i]
		}
	')"

	[[ -n "$list" ]] || die "no Wi‑Fi networks found"

	print_wifi_list
	read -r -p "Pick Wi‑Fi by SSID (or number): " choice
	choice="${choice:-}"
	[[ -n "$choice" ]] || die "no selection"

	if [[ "$choice" =~ ^[0-9]+$ ]]; then
		echo "$list" | awk -v n="$choice" 'NR==n{print; exit}'
	else
		# Match SSID exactly (first occurrence).
		echo "$list" | awk -F'|' -v ssid="$choice" '$1==ssid{print; exit}'
	fi
}

is_open_security() {
	local sec="$1"
	[[ -z "$sec" || "$sec" == "--" ]]
}

connect_wifi() {
	local selected ssid sec pw_args=()
	selected="$(pick_wifi_from_nmcli)"
	[[ -n "$selected" ]] || die "selection not found in scan results"
	ssid="${selected%%|*}"
	sec="${selected#*|}"

	info "connecting to SSID: $ssid"

	if ! is_open_security "$sec"; then
		read -r -s -p "Wi‑Fi password for \"$ssid\": " pw
		echo
		[[ -n "${pw:-}" ]] || die "empty password"
		pw_args=(password "$pw")
	fi

	# Prefer device if present.
	local wifi_dev
	wifi_dev="$(nmcli -t -f DEVICE,TYPE,STATE dev status | awk -F: '$2=="wifi"{print $1; exit}')"
	if [[ -n "${wifi_dev:-}" ]]; then
		nmcli dev wifi connect "$ssid" ifname "$wifi_dev" "${pw_args[@]}"
	else
		nmcli dev wifi connect "$ssid" "${pw_args[@]}"
	fi
}

detect_captive_portal_location() {
	# Returns Location URL if HTTP is being redirected; empty otherwise.
	curl -sSIL --max-time 8 "$NEVERSSL_URL" 2>/dev/null | awk 'BEGIN{IGNORECASE=1} /^Location:/{sub(/\r$/,""); print $2; exit}'
}

print_proxy_instructions() {
	cat >&2 <<'EOF'

Captive portal detected.

If you only have SSH/CLI access, the usual way to sign in is to route a local browser
through this machine using an SSH SOCKS proxy:

  1) On *your laptop*:
       ssh -D 1080 <user>@<this-host>

  2) In your browser network/proxy settings:
       SOCKS5 proxy: 127.0.0.1 port 1080
       (enable "proxy DNS" / "remote DNS" if the option exists)

  3) In that browser, open:
       http://neverssl.com

Sign in + check the box, then come back here. This script will keep retrying until the
portal stops intercepting and HTTPS works.

EOF
}

wait_for_internet_or_portal_login() {
	local portal_loc=""

	while true; do
		if has_internet; then
			info "internet OK (HTTPS reachable)"
			return 0
		fi

		portal_loc="$(detect_captive_portal_location || true)"
		if [[ -n "$portal_loc" ]]; then
			info "still behind captive portal (redirect -> $portal_loc)"
			print_proxy_instructions
		else
			info "no HTTPS yet; retrying (may be firewall/route/DNS)"
		fi

		sleep "$SLEEP_SECS"
	done
}

main() {
	need_cmd nmcli
	need_cmd ip
	need_cmd awk
	need_cmd curl

	if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
		# We need root for route tweaks; keep env vars for configurable URLs.
		exec sudo -E "$0" "$@"
	fi

	# If we already have internet, just show status and exit.
	if has_internet; then
		info "internet already reachable"
		ip -4 route show default || true
		exit 0
	fi

	# If Wi‑Fi isn't connected, offer interactive selection + connect.
	local wifi_state
	wifi_state="$(nmcli -t -f DEVICE,TYPE,STATE dev status | awk -F: '$2=="wifi"{print $3; exit}')"
	if [[ "$wifi_state" != "connected" ]]; then
		nmcli radio wifi on >/dev/null 2>&1 || true
		connect_wifi
	fi

	# Prefer Wi‑Fi for this runtime session so portal/login works from CLI.
	prefer_wifi_routes_runtime

	info "default route device: $(default_route_dev || echo "unknown")"
	ip -4 route show default || true

	# If we’re on a captive portal, we can’t complete it purely with curl reliably.
	# Hang and keep retrying until portal is cleared and HTTPS works.
	wait_for_internet_or_portal_login
}

main "$@"

