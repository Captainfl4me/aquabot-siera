# Repos SiERA pour Challenge Aquabot

Ce repos contient le code source pour le challenge Aquabot.

## Installation

Pour installer le noeud ROS il suffit de lancer les commandes suivantes:

```bash
cd ~/vrx_ws/src
git clone git@github.com:Captainfl4me/aquabot-siera.git
colcon build --merge-install
. install/setup.bash
```

## Bonne pratique GitHub

Pour éviter les conflits il faut créer une nouvelle branche Git quand on développe une nouvelle fonctionnalitée.

Ensuite pour merge les mises à jour on crée une pull-request sur la branche master.
