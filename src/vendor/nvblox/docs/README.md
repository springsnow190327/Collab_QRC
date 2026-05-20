# `nvblox` Dox - Developer Guide

To build the `nvblox` docs locally follow the following instructions.

Enter the `nvblox` docker.

```
./docker/run_docker.sh
```

Deactivate the `nvblox` venv

```
deactivate
```

Install a `git-lfs` for downloading images used in the docs.

```
sudo apt-get update && sudo apt-get install git-lfs
```

Create a `venv` and install the dependencies

```
python3 -m venv venv_docs
source venv_docs/bin/activate
cd ./docs
python3 -m pip install -r requirements.txt
```

To make the current version of docs

```
make html
```

To view the docs, navigate to `nvblox/docs/_build/current/html/index.html`, and double-click.

To make the multi version docs. Note that this will only build docs for the set branches, such
as release, main etc. Only docs committed to these branches will be reflected.

```
make multi-docs
```

To view the multi version docs, navigate to `nvblox/docs/_build/index.html`, and double-click.
