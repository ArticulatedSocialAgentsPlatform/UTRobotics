// 0- no pot
// 1- pot1
// 2- pot2
// 3- pot3

void pins_act(){
  int i;
  int cur;
  
  for (i = 0; i<6; i += 1){
    
    cur = analogRead(i);
    if (cur < 200){
      myPins[i] = (String)0;
    } else if (cur < 280){
      myPins[i] = "H";
    } else if (cur < 550){
      myPins[i] = "M";
    } else {
      myPins[i] = "L";
    }
    
  }
  
}
