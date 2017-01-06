// 1 = blocks
// 0 = no blocks

void reeds_init(){
  pinMode(7, INPUT);
  pinMode(8, INPUT);
}

void reeds_act(){
  reedLeft = digitalRead(8);
  reedRight = digitalRead(7);
}

