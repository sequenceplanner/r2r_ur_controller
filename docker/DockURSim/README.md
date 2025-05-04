# DockURSim forked from https://github.com/ahobsonsayers/DockURSim
- Updated to version URSim_Linux-5.17.0.128818.tar.gz
- Better vnc client (https://github.com/linuxserver/docker-baseimage-kasmvnc)
- Autostarts robot (no real need to view gui)

# Running
```bash
docker build . -t dockursim
docker run -d -e ROBOT_MODEL=UR3 --name=dockursim -p 3000:3000 -p 29999:29999 -p30001-30004:30001-30004 dockursim
```
