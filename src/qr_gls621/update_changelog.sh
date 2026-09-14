#!/bin/bash

# Ensure the script runs inside a Git repository
if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "Error: This script must be run inside a Git repository."
    exit 1
fi

# Get the latest Git tag (or default to initial commit)
LATEST_TAG=$(git describe --tags --abbrev=0 2>/dev/null || echo "")
if [ -z "$LATEST_TAG" ]; then
    echo "No previous tag found. Using all commit history."
    COMMIT_RANGE="--all"
else
    COMMIT_RANGE="$LATEST_TAG..HEAD"
    echo "Generating changelog from $LATEST_TAG to HEAD..."
fi

# Define the output changelog file
CHANGELOG_FILE="CHANGELOG.rst"

# Get the current date
CURRENT_DATE=$(date +%Y-%m-%d)

# Get the latest tag or default to 0.1.0
NEW_VERSION=$(git describe --tags --abbrev=0 2>/dev/null || echo "v0.1.0")

# Get the current branch name
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

# Get the latest commit hash and message
LATEST_COMMIT=$(git log -1 --pretty=format:"%h - %s")

# Generate commit logs without commit hashes
COMMIT_LOGS=$(git log --pretty=format:"- %s" $COMMIT_RANGE)

# Append the changelog
if [ ! -f "$CHANGELOG_FILE" ]; then
    echo "Creating $CHANGELOG_FILE..."
    echo "# Changelog" > "$CHANGELOG_FILE"
fi

{
    echo "#-----------------------------------"
    echo "# Version $NEW_VERSION ($CURRENT_DATE)"
    echo "-----------------------------------"
    echo "**Branch:** $CURRENT_BRANCH"
    echo "**Latest Commit:** $LATEST_COMMIT"
    echo ""
    echo "$COMMIT_LOGS"
} >> "$CHANGELOG_FILE"

# Confirm completion
echo "Changelog updated: $CHANGELOG_FILE"

# Update version in package.xml to match the Git tag
PACKAGE_XML="package.xml"

if [ -f "$PACKAGE_XML" ]; then
    echo "Updating version in $PACKAGE_XML to $NEW_VERSION"
    # Update the version tag inside the package.xml
    sed -i "s|<version>.*</version>|<version>$NEW_VERSION</version>|" "$PACKAGE_XML"
    echo "Updated $PACKAGE_XML to version $NEW_VERSION"
else
    echo "Error: $PACKAGE_XML not found!"
    exit 1
fi

# Commit the changes to package.xml
# git add "$PACKAGE_XML"
# git add CHANGELOG.rst
# git commit -m "Update version to $NEW_VERSION in package.xml"
# git push origin "$CURRENT_BRANCH"

# Print completion message
echo "Changelog and package.xml updated successfully."
