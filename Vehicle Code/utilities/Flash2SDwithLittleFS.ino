

/*
 * Test the following:
 * 
 * 1- read file
 * 2- create file
 * 3- append to file
 * 4- delete file
 * 5- print free space
 * 6- list directory
 * 7- Erase Flash Memory
 * 8- Speed test
 * 9- Copy all files to SD
 * 
 * This works with LittleFS and Teensyduino 1.54 beta 10
 * 
 * Use 9 after a flight to copy all files to SD Card on Teensy 4.1
 * 
 * 
 */



// just for SD
  #include "SD.h"
  #include <SPI.h> 
  File tempFile;


#include <LittleFS_NAND.h>
#include <stdarg.h>

uint64_t fTot, totSize1;

LittleFS_QSPIFlash myfs;
File file, file1, file3;



char buf[512] = "";
char fname1[32] = "/flightx.txt";
char fname2[32] = "/flighty.txt";
char fname3[32] = "/flightz.txt";
int szLen = strlen( buf );




void setup() {




  delay(2000);
  Serial.begin(115200);
  delay(2000);

  Serial.println("LittleFS Start"); delay(5);
  if (!myfs.begin()) {
    Serial.println("Error starting spidisk");
    while (1) ;
  }

  //========================  Total size
  Serial.printf("TotalSize (Bytes): %d\n", myfs.totalSize());

  //========================  List directory
  printDirectory();
  
  //========================  Read file
  Serial.println();






  
  Serial.println();
  Serial.println(">>>>  1-read, 2-write1, 3-write2, 4-delete, 5-free space, 6-directory, 7-erase, 8-speed test, 9-copy to SD");
  Serial.println();
}

void loop() {
  char chIn = 255;
  if ( Serial.available() ) {
    do {
      if ( chIn != '2' && chIn != '1' && chIn != '3' && chIn != '4' && chIn != '5' && chIn != '6' && chIn != '7' && chIn != '8' && chIn != '9' && chIn != 'x')
        chIn = Serial.read();
      else
        Serial.read();
    }
    while ( Serial.available() );
  }
  if ( chIn == '1' ) {
    readFile();
    Serial.println();
  }
  if ( chIn == '2' ) {
    writeFile();
    Serial.println();
  }
  if ( chIn == '3' ) {
    appendFile();
    Serial.println();

    
  }
  if ( chIn == '4' ) {
    deleteFile(); 
    printDirectory();
    
  }
  if ( chIn == '5' ) {
    freeSpaces();  
        
  }
  if ( chIn == '6' ) {
    printDirectory();
    
  }
  if ( chIn == '7' ) {

    eraseFlash();    
    printDirectory();

  }    

  if ( chIn == '8' ) {

    speedTest();    
  }    

  if ( chIn == '9' ) {

    copy2SD();    
  }  

    if ( chIn == 'x' ) {

    readNone();    
  }  
  
}


void speedTest() {
  // 1.1ms with open/close
  //  .132ms with no open/close
  
  
  Serial.println();
  Serial.println("Starting speed test:");
  delay(100);
  strcpy(buf,"Line 1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,Line 1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,Line 1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10");
  elapsedMicros startTest;

  for(int i=0; i <= 5000; i++) {
    file = myfs.open(fname1, FILE_WRITE);
    file.println(buf);
    file.close();
    
  }
 
   unsigned long theResult = startTest;
   Serial.print("Speed test complete: ");
   Serial.print(theResult);
   Serial.print("   per write = ");
   Serial.println(theResult / 5000);
   Serial.println();
 
}



void freeSpaces() {

  Serial.printf("Disk Usuage:\n");
  unsigned long bFree = 0;
  bFree = myfs.totalSize() - myfs.usedSize();
  Serial.printf("Bytes Used: %llu, Bytes Total:%llu\n", myfs.usedSize(), myfs.totalSize());
  Serial.print("Bytes Free: ");
  Serial.println(bFree);

}


void deleteFile() {
  Serial.println();
  Serial.println("Removing file1");
  myfs.remove(fname1);

}

void eraseFlash() {
  Serial.println();
  Serial.println("formatting flash...");
  myfs.quickFormat();
  //myfs.lowLevelFormat('.');
  

}

void theDirectory() {
  Serial.println();
  Serial.println("Removing file1");
  myfs.remove(fname1);

}

void writeFile() {
  Serial.println();
  strcpy(buf,"Flight 33,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15");
  Serial.println("Writing to file1:  ");
  Serial.println(buf);
  //myfs.mkdir("structureData1");
  file = myfs.open(fname1, FILE_WRITE);
  delay(10);
  file.println(buf);
  file.close();

}

void appendFile() {
  Serial.println();
  strcpy(buf,"Flight2 33,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15");
  Serial.println("Writing to file2:  ");
  Serial.println(buf);
  //myfs.mkdir("structureData1");
  file = myfs.open(fname2, FILE_WRITE);
  delay(10);
  file.println(buf);
  file.close();

}

void readFile() {


  Serial.println();
  Serial.println("Reading from file =====================");
  char buf2[1];
  //file = myfs.open(fname1, FILE_READ);
  file = myfs.open("flight1.txt", FILE_READ);
  while(file.available()) {
   file.read(buf2, 1);
   Serial.print(buf2[0]);
  }
  file.close();
  
}

void readNone() {


  Serial.println();
  Serial.println("Reading from no file =====================");
  char buf2[1];
  file = myfs.open("launch2.txt", FILE_READ);
  int fx = 0;
  while(file.available()) {
   file.read(buf2, 1);
   Serial.print(buf2[0]);
   fx++;
  }
  file.close();
  if(fx == 0) Serial.println("No file");
  
}


void copy2SD() {

    // Initializing the SD Card
    if(!SD.begin(BUILTIN_SDCARD)){
        Serial.println(F("SD Card Mount Failed"));
    } else {
        Serial.println(F("Good SD Card mounted"));
    }
   uint64_t fSize = 0;
  // Cycle through directory
      File mdir;
      mdir = myfs.open("/");
    while (true) {  
      File entry =  mdir.openNextFile();
      if (! entry) {
        // no more files
        totSize1 += fSize;
        break;
      }
      
      if (entry.isDirectory()) {
        Serial.print("Skipping Directory: ");
        Serial.println(entry.name());
      } else {
        Serial.print("Copying File ");
        Serial.print(entry.name());
        Serial.print("   Size = ");
        fSize = entry.size();      
        Serial.println(fSize);
        // Copy it to SD
        // Delete and open from SD
        char sName[30];
        strcpy(sName,"/");  
        strcat(sName,entry.name());
        SD.remove(sName);
        tempFile = SD.open(sName, FILE_WRITE);
        // Open for reading and writing from flash
        char buf2[1];
        file = myfs.open(entry.name(), FILE_READ);
        while(file.available()) {
         file.read(buf2, 1);
         tempFile.print(buf2[0]);
        }
        file.close();
        tempFile.close();        
      }
      entry.close();
    }
    
}


void printDirectory() {

  Serial.println("printDirectory\n--------------");
  printDirectory(myfs.open("/"), 0);
  Serial.println();
}


void printDirectory(File dir, int numTabs) {
  //dir.whoami();
  uint64_t fSize = 0;
  uint32_t dCnt = 0, fCnt = 0;
  if ( 0 == dir ) {
    Serial.printf( "\t>>>\t>>>>> No Dir\n" );
    return;
  }
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      Serial.printf("\n %u dirs with %u files of Size %u Bytes\n", dCnt, fCnt, fSize);
      fTot += fCnt;
      totSize1 += fSize;
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }

    if (entry.isDirectory()) {
      Serial.print("DIR\t");
      dCnt++;
    } else {
      Serial.print("FILE\t");
      fCnt++;
      fSize += entry.size();
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println(" / ");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
    //Serial.flush();
  }
}
