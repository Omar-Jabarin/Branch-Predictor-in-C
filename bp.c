/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <stdlib.h>
#include <math.h>
#define targetSize 30

enum predState {SNT = 0, WNT = 1 , WT = 2 , ST = 3};

enum shareType{no_share =0, low_share = 1, mid_share = 2};

bool lastPrediction;
bool isLastAHit = false;

uint32_t getBtbIndex(uint32_t pc);
uint32_t getTagFromPc(uint32_t pc);
uint32_t extractBits(uint32_t number, unsigned int begin, unsigned int end);
uint32_t getTableIndex(uint32_t pc, uint32_t historyRegister);
void updateHistoryRegister(bool taken, uint32_t* historyRegister);
void updatePredictionTable(bool taken, unsigned int index, uint32_t* table);

typedef struct {
    uint32_t tag;
    uint32_t targetAddress;
    bool valid;
    uint32_t historyReg; //used only if branch has its own local history
    uint32_t* predictionTable; //used only if each branch has its own prediction table


} BTBEntry;


typedef struct {
    unsigned int btbSize;
    unsigned int historySize;
    unsigned int tagSize;
    bool isGlobalHist;
    bool isGlobalTable;
    int Shared;
    unsigned int fsmState;
    BTBEntry* btb;
    uint32_t GHR; //Global history register. Only relevant in that case.
    uint32_t* globalTable; // global shared prediction table. only relevant in that case.
    SIM_stats stats;
} BP;



BP* bp;

/*btb size tells how many rows the tables has.
 * historySize tells us how many bits we are using to store previous branch results
 * tagSize tells us how many bits in the instruction code we are using to distinguish branches.
 * fsmState tells us what the initial branch prediction state is(whether we jump or not in case of no information).
 * isGlobal history tells us if we're saving the history of all the branches together in one register.
 * GlobalTable tells us if all the branches are using one table or if every branch has its own table
 * in case we have a global table the Shared parameter tells us if we're using xor and on which bits before we access the table.
 * */

//init might contain mistakes and be incomplete I am not sure about it yet.
int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
            bool isGlobalHist, bool isGlobalTable, int Shared){

    bp = (BP*)malloc(sizeof(BP));
    if (bp == NULL) {
        return -1;
    }
    bp->btbSize = btbSize;
    bp->historySize = historySize;
    bp->tagSize = tagSize;
    bp->fsmState = fsmState;
    bp->isGlobalHist = isGlobalHist;
    bp->isGlobalTable = isGlobalTable;
    bp->Shared = Shared;
    bp->GHR = 0;
    bp->globalTable = NULL;

    bp->stats.flush_num = 0;
    bp->stats.br_num = 0;
    bp->stats.size = 0;

    bp->btb = (BTBEntry*)malloc(sizeof(BTBEntry) * btbSize); //creating array of BtbEntries
    if (bp->btb == NULL) {
        free(bp);
        return -1;
    }

    for (unsigned int i = 0; i < btbSize; ++i) {
        bp->btb[i].valid = false;
        bp->btb[i].historyReg = 0;

    }
    //(1U<<historySize) is basically 2^(historySize) the U is for unsigned.
    unsigned int predictionTableSize = 1U << historySize;


    //Scenario 1: global table.
    if (isGlobalTable){
        bp->globalTable = (uint32_t*)malloc(predictionTableSize * sizeof(uint32_t));
        if (!bp->globalTable){
            free(bp->btb);
            free(bp);
            return -1;
        }
        //initialize to given initial fsmState in prediction table
        for (unsigned int i= 0; i < predictionTableSize; i++) bp->globalTable[i] = fsmState;
        return 0;

    }

        //Scenario 2: local tables
    else{
        for (unsigned int i = 0; i<btbSize; i++){
            bp->btb[i].predictionTable = (uint32_t*)malloc(predictionTableSize * sizeof(uint32_t));

            if (!bp->btb[i].predictionTable){
                //freeing previously allocated tables.
                while (i-- > 0) {
                    free(bp->btb[i].predictionTable);
                }
                free (bp->btb);
                free(bp);
                return -1;
            }
            //filling predictionTable with initial fsmState
            for (unsigned int j = 0; j < predictionTableSize; j++){
                bp->btb[i].predictionTable[j] = fsmState;
            }
        }
        return 0;
    }

}

bool BP_predict(uint32_t pc, uint32_t *dst){
    unsigned int btbIndex = getBtbIndex(pc);

    //no hit, we don't predict
    if (!bp->btb[btbIndex].valid || bp->btb[btbIndex].tag!= getTagFromPc(pc)){
        *dst = pc + 4;
        lastPrediction = false;
        isLastAHit = false;
        return false;
    }
        //hit we predict
    else{
        unsigned int tableIndex;
        enum predState prediction;
        isLastAHit=true;


        //scenario 1: global hist + global table
        if (bp->isGlobalTable && bp->isGlobalHist){
            tableIndex = getTableIndex(pc, bp->GHR);
            prediction = bp->globalTable[tableIndex];
            if (prediction == SNT || prediction == WNT){
                *dst = pc + 4;
                lastPrediction = false;
                return false;
            }
            else{
                *dst = bp->btb[btbIndex].targetAddress;
                lastPrediction = true;
                return true;
            }
        }


            //scenario 2: global table + local hist
        else if(bp->isGlobalTable && !bp->isGlobalHist){
            tableIndex = getTableIndex(pc ,bp->btb[btbIndex].historyReg);
            prediction = bp->globalTable[tableIndex];
            if (prediction == SNT || prediction == WNT){
                *dst = pc + 4;
                lastPrediction = false;
                return false;
            }
            else {
                *dst = bp->btb[btbIndex].targetAddress;
                lastPrediction = true;
                return true;
            }
        }


            //scenario 3: local history and local table
        else if(!bp->isGlobalTable && !bp->isGlobalHist){
            tableIndex = getTableIndex(pc, bp->btb[btbIndex].historyReg);
            prediction = bp->btb[btbIndex].predictionTable[tableIndex];
            if (prediction == SNT || prediction == WNT){
                *dst = pc + 4;
                lastPrediction = false;
                return false;
            }
            else{
                *dst = bp->btb[btbIndex].targetAddress;
                lastPrediction = true;
                return true;
            }
        }

            //scenario 4: global history and local table
        else{
            tableIndex = getTableIndex(pc ,bp->GHR);
            prediction = bp->btb[btbIndex].predictionTable[tableIndex];

            if (prediction == SNT || prediction == WNT){
                *dst = pc + 4;
                lastPrediction = false;
                return false;
            }
            else{
                *dst = bp->btb[btbIndex].targetAddress;
                lastPrediction = true;
                return true;
            }
        }



    }


}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
    bp->stats.br_num++;
    unsigned int btbIndex = getBtbIndex(pc);
    bool isNewTag = getTagFromPc(pc) != bp->btb[btbIndex].tag;
    if (taken){
        if (!(pred_dst == targetPc))
        {
            bp->stats.flush_num++ ;
        }
    }
    else {
        if (!(pred_dst == (pc + 4)))
        {
            bp->stats.flush_num++ ;
        }
    }


    if (isLastAHit){


        bp->btb[btbIndex].targetAddress = targetPc;
        //global table global history
        if (bp->isGlobalTable && bp->isGlobalHist){
            updatePredictionTable(taken, getTableIndex(pc, bp->GHR), bp->globalTable);
            updateHistoryRegister(taken, &bp->GHR);
            return;
        }

            //global table local history
        else if(bp->isGlobalTable && !bp->isGlobalHist){
            unsigned int btbIndex = getBtbIndex(pc);
            updatePredictionTable(taken, getTableIndex(pc,bp->btb[btbIndex].historyReg), bp->globalTable);
            updateHistoryRegister(taken, &bp->btb[btbIndex].historyReg);
            return;
        }
            //local table global hist
        else if (!bp->isGlobalTable && bp->isGlobalHist){
            unsigned int btbIndex = getBtbIndex(pc);
            updatePredictionTable(taken, getTableIndex(pc, bp->GHR), bp->btb[btbIndex].predictionTable);
            updateHistoryRegister(taken, &bp->GHR);
            return;
        }
            //local table local hist
        else{
            unsigned int btbIndex = getBtbIndex(pc);
            updatePredictionTable(taken, getTableIndex(pc, bp->btb[btbIndex].historyReg), bp->btb[btbIndex].predictionTable);
            updateHistoryRegister(taken, &bp->btb[btbIndex].historyReg);
            return;
        }
    }
        //If we missed
    else{
        unsigned int btbIndex = getBtbIndex(pc);
        unsigned int predictionTableSize = 1U << bp->historySize;
        bp->btb[btbIndex].valid = true;
        bp->btb[btbIndex].tag = getTagFromPc(pc);
        bp->btb[btbIndex].targetAddress = targetPc;
        //if local tables we reset it.
        if (!bp->isGlobalTable){
            for (unsigned int i=0; i<predictionTableSize; i++) bp->btb[btbIndex].predictionTable[i] = bp->fsmState;
            //in case of local history we reset the history then update it and update relevant entry
            if (!bp->isGlobalHist) {
                bp->btb[btbIndex].historyReg = 0;
                unsigned int tableIndex = getTableIndex(pc, bp->btb[btbIndex].historyReg);
                updatePredictionTable(taken, tableIndex, bp->btb[btbIndex].predictionTable);
                updateHistoryRegister(taken, &bp->btb[btbIndex].historyReg);
                return;

            }

                //in case of global history we update the global history and relevant table entry
            else {

                unsigned int tableIndex = getTableIndex(pc, bp->GHR);
                updatePredictionTable(taken, tableIndex, bp->btb[btbIndex].predictionTable);
                updateHistoryRegister(taken, &bp->GHR);
                return;
            }
        }



            //in case of global table
        else{
            //in case of global history
            if (bp->isGlobalHist){
                unsigned int tableIndex = getTableIndex(pc,bp->GHR);
                updatePredictionTable(taken, tableIndex, bp->globalTable);
                updateHistoryRegister(taken, &bp->GHR);
                return;
            }
                //in case of local history
            else{
                if(isNewTag) bp->btb[btbIndex].historyReg = 0;
                unsigned int tableIndex = getTableIndex(pc, bp->btb[btbIndex].historyReg);
                updatePredictionTable(taken, tableIndex, bp->globalTable);
                updateHistoryRegister(taken,&bp->btb[btbIndex].historyReg);
                return;
            }

        }

    }


}

void BP_GetStats(SIM_stats *curStats){
    curStats->br_num = bp->stats.br_num;
    curStats->flush_num = bp->stats.flush_num;


    if (!bp->isGlobalHist && !bp->isGlobalTable) // LOCAL HISTORY + LOCAL TABLE
    {
        curStats->size = (bp->btbSize)*(1 + bp->tagSize+targetSize + 2* pow(2,bp->historySize) + (bp->historySize));
    }
    if (bp->isGlobalHist && !bp->isGlobalTable) // GLOBAL HISTORY + LOCAL TABLE
    {
        curStats->size = (bp->btbSize)*(1 + bp->tagSize+targetSize + 2* pow(2,bp->historySize)) + (bp->historySize);
    }
    if (!bp->isGlobalHist && bp->isGlobalTable) // LOCAL HISTORY + GLOBAL TABLE
    {
        curStats->size = (bp->btbSize)*(1 + bp->tagSize+targetSize + bp->historySize) + 2* pow(2,bp->historySize);
    }
    if (bp->isGlobalHist && bp->isGlobalTable) // GLOBAL HISTORY + GLOBAL TABLE
    {
        curStats->size=(bp->btbSize)*(1 + bp->tagSize+targetSize) + (bp->historySize + 2* pow(2,bp->historySize));
    }

    if (!bp->isGlobalTable){
        for (unsigned int i = 0; i<bp->btbSize; i++) {
            if(bp->btb && bp->btb[i].predictionTable) free(bp->btb[i].predictionTable);
        }
    }

    if(bp->btb)
    {
        free(bp->btb);

    }
    if(bp->globalTable)
    {
        free(bp->globalTable);
    }
    free(bp);
    return;
}
//calculates the index in the btb based on pc and btb's size.
uint32_t getBtbIndex(uint32_t pc) {

    uint32_t count = log2(bp->btbSize);

    pc = pc>>2U; //remove 2 lowest bits from pc

    uint32_t mask = (1U << count) - 1; // select lowest count bits from pc

    return pc & mask; //bitwise and
}

uint32_t getTagFromPc(uint32_t pc){
    uint32_t count = (uint32_t)log2(bp->btbSize) + 2;

    pc = pc>>count; //remove count lowest bits from pc

    uint32_t mask =(1U << bp->tagSize) -1; //select tagSize amount lowest bits from pc.

    return pc & mask;

}

//returns selected bit from index begin to end inclusive and 0 outside example extractBits(228,2,7) returns 57
unsigned int extractBits(uint32_t number, unsigned int begin, unsigned int end) {
    if (end == 31 && begin ==0) return number;

    uint32_t mask = (1U << (end - begin + 1)) - 1;

    return (number >> begin) & mask;
}


uint32_t getTableIndex(uint32_t pc, uint32_t historyRegister){

    if (!bp->isGlobalTable) return historyRegister;
    if(!bp->Shared) return historyRegister;
    else{
        switch (bp->Shared){

            case low_share:{
                uint32_t lowPcBits = extractBits(pc,2, bp->historySize+1);
                uint32_t resultXOR = lowPcBits ^ historyRegister;
                return extractBits(resultXOR, 0, bp->historySize-1);

            }
            case mid_share:{
                uint32_t midPcBits = extractBits(pc,16, bp->historySize+15);
                uint32_t resultXOR = midPcBits ^ historyRegister;
                return extractBits(resultXOR, 0, bp->historySize-1);

            }

            default:
                return historyRegister;
        }
    }
}

void updateHistoryRegister(bool taken, uint32_t* historyRegister){
    uint32_t temp = *historyRegister;
    temp = temp<<1U;
    temp = extractBits(temp, 0, bp->historySize - 1);
    temp |= (taken ? 1U : 0U);
    *historyRegister = temp;
}

void updatePredictionTable(bool taken, unsigned int index, uint32_t* table){
    switch (table[index]) {

        case SNT:
            if (taken) table[index] = WNT;
            return;

        case WNT:
            if (taken) table[index] = WT;
            else table[index] = SNT;
            return;

        case WT:
            if (taken) table[index] = ST;
            else table[index] = WNT;
            return;

        case ST:
            if (!taken) table[index] = WT;
            return;

    }
}