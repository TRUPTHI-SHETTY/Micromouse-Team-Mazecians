# Micromouse-Team-Mazecians

Maze Solving Robot Challenge:

Code Overview:

Initially flag=true for normal run, 

If value stored is 50 at location 0 of EEPROM, execute final run code and set flag=false to avoid normal run.

If key='2' is pressed on keypad module, write initially 0 and then 255 to all locations of EEPROM to avoid any other junk values.

In case of Normal Run:
    1. Read values from front,left and right IR sensors

    2. Based on the values from sensors,assign wallType variable as shown below:
        6 - All walls
        31 - leftWall & rightWall
        30 - leftWall & frontWall
        10 - rightwall & front wall
        3 - leftWall
        1 - rightwall
        0 - frontwall
        4 - No walls

    3. To satisfy a game rule: "Avoid visiting start cell(0,0) again", check if it detects (xvar,yvar)=(0,1) or (1,0) cell again,new wallType will be assigned to it which helps to turn in such a way that it will not visit (0,0).

    4. Make turns based on below logic:
        // 30  leftWall & frontWall(1 Choice:Turn Right)
        // 10 rightwall & front wall(1 Choice:Turn Left)

        //temp[x][y] is 3 //forward when rightWall
        //temp[x][y] is 6 //leftTurn when rightWall
        //temp[x][y] is 4 // rightTurn when leftWall
        //temp[x][y] is 5 //forward when leftWall
        //temp[x][y] is 1  //leftTurn when frontWall
        //temp[x][y] is 2  //rightTurn when frontWall


        // 1 rightwall (2 choice: LeftTurn or forward)
        //First time rightwall,move forward(3),Second time same rightwall,take leftTurn(6)
        //Here for temp[x][y]==5 is given for a case where if already in that cell mouse has gone forward with leftWall,
        //and now you are detecting the cell again as having right wall(This case is when mouse is coming back to the same cell in opposite direction).
        //As it is rightwall in the second time(2 Choice: LeftTurn or Forward)--> But choosing forward will take the mouse in already travelled path.
        //So,to avoid repetition in this case leftTurn is used.


        // 3 leftwall (2 choice: rightTurn or forward)
        //First time leftwall,move forward(5),Second time same leftwall,take rightTurn(4)
        //Here for temp[x][y]==3 is given for a case where if already in that cell mouse has gone forward with rightWall,
        //and now you are detecting the cell again as having left wall(This case is when mouse is coming back to the same cell in opposite direction).
        //As it is leftwall in the second time(2 Choice: rightTurn or Forward)--> But choosing forward will take the mouse in already travelled path.
        //So,to avoid repetition in this case rightTurn is used.

        // 0 frontwall (2 choice: Left or Right)
        //First time frontwall,take leftTurn(1),Second time same frontwall,take rightTurn(2)
        //Here for temp[x][y]==3 is given for a case where if already in that cell mouse has gone forward with rightWall,
        //and now you are detecting the cell again as having front wall(This case is when mouse is coming back to the same cell in opposite direction).
        //As it is frontwall in the second time(2 Choice: rightTurn or leftTurn)--> But choosing leftTurn will take the mouse in already travelled path.
        //So,to avoid repetition in this case rightTurn is used.

        // 4  No walls (Multiple choice: Choose Forward Always)
        // 31 leftWall & rightWall (1 Choice:forward)
        // 6  All walls(1 Choice:BackwardTurn)

    5. Update direction based on direction of turn taken and the direction before turn:

       //directionValue:(Top:0,Right:1,Bottom:2,Left:3)

    6. Store new direction in directionStore Array for corresponding coordinates

    7. Update x,y coordinates based on the updated new direction

    8. Check if pattern of final 3 cells is detected and 
        if yes, 
            stop at 4th final cell and storeFinalDirection of all cells(Based on Current direction and Stored Direction-->Derive final direction of the cell. 
            For the next cell,take final direction of previous cell as current direction and also use stored direction to derive its final direction.
            Repeat same logic for all cells), 
        else 
            repeat all steps starting from Step 1 for every new (x,y) cell.

In case of Final Run:
    Make movements based on stored final Directions of all cells and stop when final coordinates (fx,fy) are reached.