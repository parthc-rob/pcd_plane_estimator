#!/usr/bin/env bash

# Fail on first error.
set -e

echo "Setting up user parameters..."
# Add basic user
#USERNAME=user

useradd -m $USERNAME
echo "$USERNAME:$USERNAME" | chpasswd

#adduser --disabled-password --gecos '' ${USERNAME}
usermod --shell /bin/bash $USERNAME
usermod -aG sudo ${USERNAME}
#echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME
#chmod 0440 /etc/sudoers.d/$USERNAME

# Replace 1000 with your user/group id
usermod  --uid 1000 $USERNAME
groupmod --gid 1000 $USERNAME

echo """
ulimit -c unlimited
""" >> /home/${USERNAME}/.bashrc

#Fix for qt and X server errors
echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc && \
# cd to home on login
echo "cd" >> /home/$USERNAME/.bashrc

# Setting language
LANG="en_US.UTF-8"
echo "export LANG=\"en_US.UTF-8\"" >> /home/$USERNAME/.bashrc

