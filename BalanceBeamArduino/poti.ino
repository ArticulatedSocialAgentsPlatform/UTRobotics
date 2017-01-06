// potmeter
// connections: GRD, D6/A7, +5V
// states: 1- left, 2- center, 3- right (child's view)


void poti_act(){
  poti_cur = analogRead(7);
  
  //Serial.println(poti_cur);
  //poti_cur = smooth(poti_cur, 0.5, poti_prev);
  poti_prev = poti_cur;
  if (poti_cur < poti_center+poti_thresh && poti_cur > poti_center-poti_thresh) {
    poti_state = 2;
  } else if (poti_cur >= poti_center+poti_thresh) {
    poti_state = 1; // change to 3 if mirrored
  } else {
    poti_state = 3; // change to 1 if mirrored 
  }
}

int smooth(int currVal, float filterVal, int prevVal) {
  float smoothedVal = (currVal * (1 - filterVal)) + (prevVal  *  filterVal);
  return (int)smoothedVal;
}
