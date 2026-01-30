#!/bin/bash
#
# GitHub Repository Setup Helper
# Assists with GitHub authentication and repository cloning for DCM fleet deployment
#

set -e

echo "=== GitHub Repository Setup ==="

# Check if gh CLI is installed
if ! command -v gh &> /dev/null; then
    echo "GitHub CLI (gh) is not installed."
    echo "Installing GitHub CLI..."

    # Install gh CLI
    type -p curl >/dev/null || sudo apt install curl -y
    curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
    sudo chmod go+r /usr/share/keyrings/githubcli-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null
    sudo apt update
    sudo apt install gh -y
fi

echo ""
echo "GitHub CLI version: $(gh --version | head -n1)"

# Check authentication status
echo ""
echo "Checking GitHub authentication status..."
if gh auth status &> /dev/null; then
    echo "Already authenticated with GitHub."
    gh auth status
else
    echo "Not authenticated with GitHub."
    echo ""
    echo "Starting GitHub authentication..."
    echo "You will be prompted to authenticate via browser or token."
    echo ""
    gh auth login
fi

# Verify authentication
echo ""
echo "Verifying authentication..."
if gh auth status &> /dev/null; then
    echo "GitHub authentication successful!"
else
    echo "Error: GitHub authentication failed."
    exit 1
fi

# Test repository access
echo ""
echo "Testing repository access..."
REPOS=(
    "Ayanda43/dcm-fleet-deployment"
    "Ayanda43/tsam-dcm"
)

for repo in "${REPOS[@]}"; do
    if gh repo view "$repo" &> /dev/null; then
        echo "  [OK] Can access: $repo"
    else
        echo "  [WARN] Cannot access: $repo (may be private or non-existent)"
    fi
done

echo ""
echo "=== GitHub Setup Complete ==="
echo ""
echo "You can now clone repositories using:"
echo "  gh repo clone Ayanda43/dcm-fleet-deployment"
echo "  gh repo clone Ayanda43/tsam-dcm"
