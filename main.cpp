#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SdFat.h>
#include <string>
#include <fstream>
#include <iostream>
#include <stdint.h>

using namespace std;

SdFs SD;
FsFile file;
//CAN1 - refers to CAN controller
//RX_SIZE_256 - space for 256 messages that can be stored while waiting
//TX_SIZE_16 - obj can queue 16 outgoing CAN messages
FlexCAN_T4 <CAN1, RX_SIZE_256, TX_SIZE_16> canBUS;
FsFile dbcFile;

//initialize SD & CAN Sensor
void setup(){
  Serial.begin(9600);
  delay(1000);

  dbcFile = SD.open("MF13Beta(Untested 2.0).dbc", FILE_READ);
    if (!dbcFile) {
      Serial.println("Failed to open DBC file!");
      while (1);
    }

  //Initialize SD Card
  if (!SD.begin(SdioConfig(FIFO_SDIO))) {
    Serial.println("SD initialization failed!");
    while (1); 
  }

  file = SD.open("RDAQ.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Error opening file for writing!");
    while (1); 
  }

  canBUS.begin();
  canBUS.setBaudRate(1000000); 

  //creates message queue for recieved frames
  canBUS.enableFIFO(); 

  //alerts when new frame arrives
  canBUS.enableFIFOInterrupt();

  //callback function for recieved messages
  canBUS.onReceive(canSniff); 

  //print mailbox status
  canBUS.mailboxStatus(); 
}


void loop() {
  canBUS.events();
}

void canSniff(const CAN_message_t & incomingMsg){
  Serial.print("OVERRUN: "); 

  //determine if a buffer (temp storage) overrun occured
  Serial.print(incomingMsg.flags.overrun); 
  Serial.print("LEN: "); 

  //message length in bytes
  Serial.print(incomingMsg.len); 
  Serial.print("EXT: "); 

  //extended identifier flag
  Serial.print(incomingMsg.flags.extended); 
  Serial.print("EIF: "); 

  //timestamp of message reception
  Serial.print(incomingMsg.timestamp); 
  Serial.print("ID: "); 

  //message identifier
  Serial.print(incomingMsg.id, HEX); 
  Serial.print("Buffer: ");
  
  //print message payload by byte
  for ( uint8_t i = 0; i < incomingMsg.len; i++ ) {

    //prints each byte as hexadecimal
    Serial.print(incomingMsg.buf[i],HEX); 
    Serial.print(" ");
  } 
  Serial.println();

  //print data to file

  char idHexStr[10];
  // convert numeric ID to hex string
  sprintf(idHexStr, "%lX", incomingMsg.id);  
  //convert hexadecimal to decimal
  int idDecimal = std::stoi(String(idHexStr).c_str(), nullptr, 16);  
  file.print("ID: ");

  //match decimal to string
  std::string line;
  std::string target = "BO_ " + std::to_string(idDecimal) + " ";
  line.clear();
  char c;
  bool message = false;
  while (dbcFile.available()) {
    c = dbcFile.read();
    if (!line.empty()) {
      // process the completed line
      if (line.compare(0, target.size(), target) == 0) {
        size_t nameStart = line.find(' ', target.size());
        size_t nameEnd   = line.find(':', nameStart);
        if (nameStart != std::string::npos && nameEnd != std::string::npos) {
          std::string name = line.substr(nameStart + 1, nameEnd - nameStart - 1);
          file.print(name.c_str());
          message = true;
        }
      }
      else if (message == true && (line.compare(0, 3, "SG_") == 0)) {
        // 1. signal name text between SG and :
        size_t start  = line.find("SG_") + 3;
        size_t colon  = line.find(':', start);
        string signal = line.substr(start, colon - start);
        signal.erase(remove(signal.begin(), signal.end(), ' '), signal.end());
        // 2. start bit and bit length
        size_t pipe = line.find('|', colon);
        size_t at   = line.find('@', pipe);
        int start_bit = stoi(line.substr(colon + 1, pipe - colon - 1));
        int end_bit   = stoi(line.substr(pipe + 1, at - pipe - 1));
        // 3. endian + signed
        char order = line[at + 1];
        char sign  = line[at + 2];
        bool isLittle = (order == '0');
        bool isSigned = (sign == '-');
        // 5. scale and offset
        size_t openParen  = line.find('(');
        size_t closeParen = line.find(')');
        size_t comma      = line.find(',');
        float scale  = stof(line.substr(openParen + 1, comma - openParen - 1));
        float offset = stof(line.substr(comma + 1, closeParen - comma - 1));
        // 6. units
        size_t quote1 = line.find('"');
        size_t quote2 = line.find('"', quote1 + 1);
        string unit = " ";
        if (quote1 != string::npos && quote2 != string::npos) {
          unit = line.substr(quote1 + 1, quote2 - quote1 - 1);
        }
        // 4. endian decoding
        uint64_t rawValue = 0;
        if (isLittle) {
          for (int i = 0; i < end_bit; i++) {
          // compute byte index containing the bit
            int byteInd = (start_bit + i) / 8;

            // compute bit within that byte
            int bitInd = (start_bit + i) % 8;

            // if bit is set in CAN buffer
            if (incomingMsg.buf[byteInd] & (1 << bitInd)) {
              // set bit i inside rawValue
              rawValue |= (uint64_t(1) << i);
            }
          }
        }
            
        if (!isLittle) {
          for (int i = 0; i < end_bit; i++) {
            // compute byte index containing the bit
            int byteInd = (start_bit - i) / 8;
            // compute bit within that byte
            int bitInd  = (start_bit - i) % 8;

            // check bit (flip inside-byte indexing)
            if (incomingMsg.buf[byteInd] & (1 << (7 - bitInd))) {
              rawValue |= (uint64_t(1) << i);
            }
          }
        }
            
        // signed/unsigned & scaling
        double physicalValue = 0;
        if (isSigned) {
          int64_t signedValue = rawValue;
          if (rawValue & (1ULL << (end_bit - 1))) {
            signedValue -= (1ULL << end_bit);
          }
          physicalValue = signedValue * scale + offset;
        } 
        else {
          physicalValue = rawValue * scale + offset;
        }

        static bool firstWrite = true;
        if (firstWrite) {
          file.println("SignalName,RawValue,PhysicalValue,Units");
          firstWrite = false;
        }

        file.print(signal.c_str());
        file.print(",");
        file.print(rawValue);
        file.print(",");
        file.print(physicalValue);
        file.print(",");
        file.println(unit.c_str());

      }
      line.clear();
    }
    else {
      line += c;
    }
  }

  //print data to file
  file.print(" ");
  file.print("Data: ");
  for (uint8_t i = 0; i < incomingMsg.len; i++){
    file.print(incomingMsg.buf[i], HEX);
    file.print(" ");
  }
  file.println();

  //send data to RAM
  file.flush();
}


