/*
 * Minimal TCP echo server (LwIP raw API).
 *
 * This intentionally avoids app-specific payload generation and simply echoes
 * received data back to the client.
 */

#include "tcp_echo_server.h"

#include "lwip/tcp.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"

typedef struct
{
  struct pbuf *pending;
} echo_conn_t;

static err_t echo_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t echo_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void  echo_err(void *arg, err_t err);

static err_t echo_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  (void)arg;
  (void)err;

  echo_conn_t *c = (echo_conn_t *)mem_malloc(sizeof(echo_conn_t));
  if (!c) return ERR_MEM;

  c->pending = NULL;

  tcp_arg(newpcb, c);
  tcp_recv(newpcb, echo_recv);
  tcp_sent(newpcb, echo_sent);
  tcp_err(newpcb, echo_err);
  tcp_setprio(newpcb, TCP_PRIO_MIN);

  return ERR_OK;
}

static void echo_close(struct tcp_pcb *tpcb, echo_conn_t *c)
{
  tcp_arg(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_err(tpcb, NULL);

  if (c)
  {
    if (c->pending) pbuf_free(c->pending);
    mem_free(c);
  }

  tcp_close(tpcb);
}

static err_t echo_flush(struct tcp_pcb *tpcb, echo_conn_t *c)
{
  while (c->pending && tcp_sndbuf(tpcb) >= c->pending->len)
  {
    struct pbuf *p = c->pending;

    err_t w = tcp_write(tpcb, p->payload, p->len, TCP_WRITE_FLAG_COPY);
    if (w != ERR_OK)
      return w;

    tcp_recved(tpcb, p->len);
    c->pending = p->next;
    p->next = NULL;
    pbuf_free(p);
  }
  return ERR_OK;
}

static err_t echo_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  echo_conn_t *c = (echo_conn_t *)arg;
  if (!c) return ERR_ARG;

  if (p == NULL)
  {
    echo_close(tpcb, c);
    return ERR_OK;
  }

  if (err != ERR_OK)
  {
    pbuf_free(p);
    return err;
  }

  /* chain into pending list */
  if (c->pending == NULL)
  {
    c->pending = p;
  }
  else
  {
    pbuf_chain(c->pending, p);
  }

  (void)echo_flush(tpcb, c);
  return ERR_OK;
}

static err_t echo_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  (void)len;
  echo_conn_t *c = (echo_conn_t *)arg;
  if (!c) return ERR_OK;
  (void)echo_flush(tpcb, c);
  return ERR_OK;
}

static void echo_err(void *arg, err_t err)
{
  (void)err;
  echo_conn_t *c = (echo_conn_t *)arg;
  if (c)
  {
    if (c->pending) pbuf_free(c->pending);
    mem_free(c);
  }
}

void tcp_echo_server_init(uint16_t port)
{
  struct tcp_pcb *pcb = tcp_new();
  if (!pcb) return;

  if (tcp_bind(pcb, IP_ADDR_ANY, port) != ERR_OK)
  {
    tcp_close(pcb);
    return;
  }

  pcb = tcp_listen(pcb);
  tcp_accept(pcb, echo_accept);
}

