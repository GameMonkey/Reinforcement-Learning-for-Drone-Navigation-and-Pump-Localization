#!/bin/bash
for i in {1..80}
do
    echo "Running experiment ${i}"
    python3 stompc.py
    
    echo "Sleepting for 10 seconds before next experiment"
    sleep 10
    
    printf "\n\n\n"
done
