# ares-main

---
## Intro to Github

### Generate SSH Key

#### For Mac/Linux:
```bash
# Generate a new SSH key
ssh-keygen -t ed25519 -C "your_email@example.com"

# Start the ssh-agent
eval "$(ssh-agent -s)"

# Add your SSH private key to the ssh-agent
ssh-add ~/.ssh/id_ed25519

# Copy the public key to clipboard (Mac)
pbcopy < ~/.ssh/id_ed25519.pub

# Or display the public key to copy manually
cat ~/.ssh/id_ed25519.pub
```

#### For Windows:
```bash
# Generate a new SSH key using Git Bash or PowerShell
ssh-keygen -t ed25519 -C "your_email@example.com"

# Start the ssh-agent
eval "$(ssh-agent -s)"

# Add your SSH private key to the ssh-agent
ssh-add ~/.ssh/id_ed25519

# Copy the public key to clipboard (Windows)
clip < ~/.ssh/id_ed25519.pub

# Or display the public key to copy manually
cat ~/.ssh/id_ed25519.pub
```

**Next Steps:**
1. Copy the public key output
2. Go to GitHub → Settings → SSH and GPG keys
3. Click "New SSH key"
4. Paste your public key and give it a descriptive title
5. Click "Add SSH key"

### Clone the Repository

```bash
git clone --recurse-submodules git@github.com:Senior-Design-Project-ARES/ares-main.git
cd <your-repo>
```

### Create a New Branch
[Please reference this for info on how to config branches](docs/Branching_README.md)
_(I worked really hard on that ReadMe pls read it pretty pls I beg you)_

**Create a new branch:**
```bash
git checkout -b <system>-<component>-<feature>-branch
```
_to get into a branch that already exist forgo the -b..._

**Switch to an existing branch:**
```bash
git checkout <system>-<component>-<feature>-branch
```

### Make Changes and Commit

**Add all changes:**
```bash
git add .
git commit -m "Message for Commit"
```

**Add specific files:**
```bash
git add script.py otherFile.cpp notes.txt
git commit -m "Message for Commit"
```
### Push to Github
```bash
git push origin <your-branch-name>
```

### Pull from Github
```bash
git checkout <branch-or-main>
git pull origin <branch-or-main>
```

### Merge Branch to Main (Local)
**Note: This is for local merging only. Use Pull Requests for GitHub merging.**

```bash
git checkout main
git pull origin main
git merge <branch-name>
```

_Resolve any conflicts and test your code with the merged branch_

---
## Developer's Guide to GitHub

### Best Practices for Collaborative Development

#### Pull Often
- **Always pull before starting work**: `git pull origin <branch or main>` before creating new branches
- **Stay up to date**: Pull from main regularly to avoid merge conflicts
- **Pull before pushing**: Ensure your branch is current with the latest changes

#### Pull Requests (PR) for Main Branch
- **Always use PRs**: Never push directly to main branch
- **Create PRs for all changes**: Even small fixes should go through PR process
- **Write clear PR descriptions**: Explain what changes were made and why
- **Request reviews**: Ask team members to review your code
- **Address feedback**: Respond to review comments and make necessary changes
- **Test before merging**: Ensure all tests pass and code works as expected

---
