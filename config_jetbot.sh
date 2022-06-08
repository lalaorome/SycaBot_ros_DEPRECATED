#!/bin/bash

sudo addgroup --system robot
sudo adduser --system --no-create-home --disabled-login --disabled-password --ingroup robot robot
adduser robot video
adduser robot gpio