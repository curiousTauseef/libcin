#!/bin/sh
echo 'Setting up the script...'

set -e

echo 'Generating Doxygen code documentation...'
make docs 2>&1 | tee doxygen.log

echo 'Cloning repo .....'
# Get the current gh-pages branch
git clone -b gh-pages https://git@$GH_REPO_REF
cd $GH_REPO_NAME

git config --global push.default simple
git config user.name "Travis CI"
git config user.email "travis@travis-ci.org"

test -d doxygen && rm -rf doxygen
mkdir doxygen

test -e .nojekyll || echo "" > .nojekyll


echo 'Copying docs to repo dir'
cp -r ../doc/html/* doxygen
cp ../doxygen.log doxygen

echo 'Copying pdf docs'

cp ../doc/latex/refman.pdf doxygen/libcin-manual.pdf

echo 'Uploading documentation to the gh-pages branch...'
git add --all
git commit -m "Deploy code docs to GitHub Pages Travis build: ${TRAVIS_BUILD_NUMBER}" -m "Commit: ${TRAVIS_COMMIT}"

git push --force "https://${GH_REPO_TOKEN}@${GH_REPO_REF}" > /dev/null 2>&1
