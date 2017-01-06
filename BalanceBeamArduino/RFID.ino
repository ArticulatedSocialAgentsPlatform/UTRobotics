
void RFID_init(){
  SPI.begin();		// Init SPI bus
  mfrc522.PCD_Init();	// Init MFRC522 card
  pinMode(13, OUTPUT);
  pinMode(3, INPUT);
}

void RFID_find(){
  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
	  return; // don't execute rest of the program is no new card is found
  }

  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) {
	  return; // get the card's ID
  }

  // Dump debug info about the card. PICC_HaltA() is automatically called.
  // store instead of dumping directly --> add identifier
  mfrc522.PICC_DumpIDToSerial(&(mfrc522.uid));
}
