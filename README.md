# Doc for CSE400 @ Syracuse University

### This project is based on [VuePress](https://vuepress.vuejs.org/)

This project uses Yarn to install VuePress and to run it. However, you might be able to use npm as well. However, here I'm only going to show how to build this notebook using yarn.



``` bash
npm install --global yarn
```
After yarn is installed, we can install VuePress using yarn:
``` bash
yarn add -D vuepress
```
Threre are two scripts that are usful in this repo, one is
``` bash
yarn docs:dev
```
This will run a server at localhost:8080, and it will automatically refresh once there's a change in code/document
the other one is:
``` bash
yarn docs:build
```
This will build the entire website and then output to <repo_folder>/docs/.vuepress/dist

