#!/bin/bash
set -e # Exit with nonzero exit code if anything fails

SOURCE_BRANCH="master"
TARGET_BRANCH="master"

# Save some useful information
REPO=`git config remote.origin.url`
SSH_REPO=${REPO/https:\/\/github.com\//git@github.com:}
SHA=`git rev-parse --verify HEAD`
echo "SSH_REPO: ${SSH_REPO}"

# Now let's go have some fun with the cloned repo
git config user.name "$GH_USER_NAME"
git config user.email "$GH_USER_EMAIL"

# Commit the "changes", i.e. the new version.
git add --all doc/
git commit -am "[ci skip] Deploy to GitHub Pages: ${SHA}"
echo "Committed changes."

# Now that we're all set up, we can push.
git push $SSH_REPO $TARGET_BRANCH
echo "Successfully pushed to ${TARGET_BRANCH}"
