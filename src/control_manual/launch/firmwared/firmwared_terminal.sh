#!/bin/bash
#   Lanzadores del firmwared Parrot-Sphinx
echo "hola $USER"
echo "Medallo 81" 
sudo systemctl start firmwared.service | sudo firmwared
