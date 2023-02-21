const unsigned MODE_MAIN =  1;
const unsigned MODE_CONNECTED = 2;

const unsigned DOOR_OPEN = 0;
const unsigned DOOR_CLOSE = 1;

const unsigned DOOR_MODE_OPEN = 0;
const unsigned DOOR_MODE_CLOSE = 1;
const unsigned DOOR_MODE_AUTO = 3;

unsigned MODE  = MODE_MAIN;

void setup(){
    

}


void loop(){
    switch (MODE)
    {
    case MODE_MAIN:
        main();
        break;

    case MODE_CONNECTED:
        connected();
        break;
    
    default:
        break;
    }

}

void main(){

}

void connected(){

}