# Branching Policy

To keep our development process clean, collaborative, and conflict-free, we will follow a clear branching strategy.

---

## General Rule

If you're developing **any feature, fix, or script** (e.g., Guidance/Navigation algorithms, Dynamics/Controls Simulations, CD&H Workflows, etc.), you **must create a branch**.  
**DO NOT** develop directly on `main`.

---

## Branch Naming Convention

Branches should reflect the **system** and **the specific feature or problem**. Use lowercase and kebab-case for readability.

**Format:**

```
<system>-<component/feature>-branch
```

**Examples:**

- `cdh-firmware-branch`
- `gnc-path-trajectory-branch`
- `gnc-simulink-model-branch`

---

## Branch Lifecycle

1. **Create a branch** off `main` before you begin work.
2. Keep branches **short-lived and focused**. Don’t sit on a branch for weeks.
3. As soon as the feature/fix is working and tested, **merge that shit to main**.
4. After review, **merge back into `main`** and **delete the branch**.

---

---

## Why This Matters

- Huge, hard-to-review merges/PRs from long-running branches
- Merge conflicts that wasted time
- Integration bugs that surfaced late in the build cycle

Think: **"merge early, merge often"** — even partial progress can be merged if it’s tested and non-breaking.

---

## Tips

- If your branch is dragging on, consider **splitting the task** into smaller, mergeable pieces, and talk to either Vishnu, YuKang, Allison, or Sam.
- If you're not sure when or how to merge, **ask** — don't let code rot on a branch.

---

Let’s push out clean, safe, and fast code!
