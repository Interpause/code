# hello-ros

learning ROS2 now

## X GUI Apps

### Linux

[`.devcontainer/docker-compose.yml`](.devcontainer/docker-compose.yml) is already configured to hijack your X server to display GUI apps. Sorry, no Wayland support.

### Windows

You need to have an X Server for Windows beforehand (yes they do exist). Then `source win-xhack.sh`, which contains a little magic to connect to said X server.

I used [X410](https://x410.dev/) because I had purchased it a long time ago while experimenting with WSL. It also has a special mode to expose to WSL2.

Why do X Servers for Windows still exist inspite of WSL2 now having the ability to show both X and Wayland GUIs? Well, for X11 SSH forwarding on Windows (as compared to VNC). Also I cannot figure out how to get WSL2's builtin display adapter to work inside containers.

## Intellisense & Other Extensions

If using VSCode, they will auto-install. I am guessing they intend for `.devcontainer` to become a standard, but clearly including extensions is non-standard. Anyways, _Pylance won't work correctly on a fresh-install_, close & reopen (or reload) VSCode at least once for it to work. Don't ask me why.
