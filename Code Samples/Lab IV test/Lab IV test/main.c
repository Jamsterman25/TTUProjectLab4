//
//  main.c
//  Lab IV test
//
//  Created by Jeremiah McCutcheon on 7/30/18.
//  Copyright © 2018 Jeremiah McCutcheon. All rights reserved.
//

#include <stdio.h>
#include <string.h>

int main() {
    struct BTDevice{
        char MAC[13];
        char NAME[17];
        char RSSI[3];
    };
    
    struct BTDevice BTVector[10];
    
    // The single quote ' is used in place of " which is what is actually recieved from the BC127 device
    
    char RXED[256] = "PENDING INQUIRY 20FABB09067C 'BC127_MODULE_4' 240404 34db INQUIRY 20FABB069JBK 'Jamoji JK' 240404 69db INQUIR_OK";
    
    unsigned int i=0;
    unsigned int new = 1;
    unsigned int BTVec = 0;
    for(i=8; i<117; i++){
        struct BTDevice temp;
        memset(temp.NAME, '\0', 16);
        switch(RXED[i]){
            case 'I':
                i += 8;
                unsigned int iMAC = 0;
                for(iMAC=0; iMAC<12; iMAC++){
                    temp.MAC[iMAC] = RXED[i];
                    i++;
                }
                break;
            case '\'':
                i++;
                unsigned int iNAME = 0;
                while(RXED[i] != '\''){
                    if(iNAME == 16){
                        i++;
                        continue;
                    }
                    temp.NAME[iNAME] = RXED[i];
                    iNAME++;
                    i++;
                }
                unsigned int iStruct = 0;
                for(iStruct = 0; iStruct < 10; iStruct++){
                    if(!strcmp(BTVector[iStruct].NAME,temp.NAME)){
                        new = 0;
                    }
                }
                if(new){
                    unsigned int iMAC = 0;
                    for(iMAC = 0; iMAC<12; iMAC++)
                        BTVector[BTVec].MAC[iMAC] = temp.MAC[iMAC];
                    
                    unsigned int iNAME = 0;
                    while(temp.NAME[iNAME]){
                        BTVector[BTVec].NAME[iNAME] = temp.NAME[iNAME];
                        iNAME++;
                    }
                    i += 9;
                    BTVector[BTVec].RSSI[0] = RXED[i];
                    i++;
                    BTVector[BTVec].RSSI[1] = RXED[i];
                    i += 3;
                    BTVec++;
                    break;
                }
                else{
                    i += 13;
                    break;
                }
                //DEAL WITH RSSI
                break;
            default:
                break;
        }
    }
    i=0;
    
    
}
