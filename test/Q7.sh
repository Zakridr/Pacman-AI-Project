#!/bin/bash

echo "UCS on test search"
python pacman.py -l testSearch -p SearchAgent -a fn=ucs,prob=FoodSearchProblem -q
echo "Astar on test search"
python pacman.py -l testSearch -p AStarFoodSearchAgent -q
echo "UCS on tiny search"
python pacman.py -l tinySearch -p SearchAgent -a fn=ucs,prob=FoodSearchProblem -q
echo "Astar on tiny search"
python pacman.py -l tinySearch -p AStarFoodSearchAgent -q
echo "UCS on tricky search"
python pacman.py -l trickySearch -p SearchAgent -a fn=ucs,prob=FoodSearchProblem -q
echo "Astar on tricky search"
python pacman.py -l trickySearch -p AStarFoodSearchAgent -q
