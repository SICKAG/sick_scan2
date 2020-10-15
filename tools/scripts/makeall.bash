#!/bin/bash
printf "\033c"
echo -e "makeall.bash: build and install sick_scan2"
./cleanup.bash
./make.bash
echo -e "makeall.bash finished."

