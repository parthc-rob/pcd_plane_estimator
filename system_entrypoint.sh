#!/bin/bash 

# fail on first error
set -e 

echo 'export LC_ALL='C'' >> ~/.bashrc

source ~/.bashrc
cd /code-sample 

exec "$@"
