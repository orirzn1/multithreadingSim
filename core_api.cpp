/* 046267 Computer Architecture - HW #4 */

#include "core_api.h"
#include "sim_api.h"

#include <stdio.h>
#include <memory>
#include <vector>
#include <iostream>

class thread
{
private:
    std::unique_ptr<tcontext> regFile;
    int cyclesToRun;
    int currentLine;
    bool halted;

public:
    thread() : regFile(new tcontext()), cyclesToRun(0), currentLine(0), halted(false)
    {
        for(int i = 0; i < REGS_COUNT; i++)
        {
            regFile.get()->reg[i] = 0;
        }
    }
    bool isReady()
    {
        return (cyclesToRun == 0);
    }
    
    void updateMemRegFile(int destReg, int src1Reg, int src2RegImm, cmd_opcode operation, bool src2IsImm)
    {
        if(operation == CMD_ADD)
            regFile->reg[destReg] = regFile->reg[src1Reg] + regFile->reg[src2RegImm];
        else if(operation == CMD_ADDI)
            regFile->reg[destReg] = regFile->reg[src1Reg] + src2RegImm;
        else if(operation == CMD_SUB)
            regFile->reg[destReg] = regFile->reg[src1Reg] - regFile->reg[src2RegImm];
        else if(operation == CMD_SUBI)
            regFile->reg[destReg] = regFile->reg[src1Reg] - src2RegImm;
        else if(operation == CMD_LOAD) // dst <- Mem[src1 + src2]  (src2 may be an immediate)
        {
            if(src2IsImm)
            {
                SIM_MemDataRead(regFile->reg[src1Reg] + src2RegImm, &regFile->reg[destReg]);
            }
            else
            {
                SIM_MemDataRead(regFile->reg[src1Reg] + regFile->reg[src2RegImm], &regFile->reg[destReg]);
            }
        }
        else if(operation == CMD_STORE) // Mem[dst + src2] <- src1  (src2 may be an immediate)
        {
            if(src2IsImm)
                SIM_MemDataWrite(regFile->reg[destReg]+src2RegImm, regFile->reg[src1Reg]);
            else
                SIM_MemDataWrite(regFile->reg[destReg]+regFile->reg[src2RegImm], regFile->reg[src1Reg]);
        }
    }
    
    tcontext getRegFile()
    {
        return *(regFile.get());
    }
    
    void setCycles(int numCycles)
    {
        cyclesToRun = numCycles;
    }
    
    void countCycle()
    {
        if(cyclesToRun > 0)
            cyclesToRun--;
    }
    
    int getLineToRead()
    {
        return currentLine;
    }
    
    void nextLine()
    {
        currentLine++;
    }
    
    bool isHalted()
    {
        return halted;
    }
    
    void halt()
    {
        halted = true;
    }
};

class core
{
private:
    std::vector<std::unique_ptr<thread>> threads;
    int currentThread;
    int numThreads;
    int storeLat;
    int loadLat;
    double numCyclesTot;
    double numInstrTot;

public:
    explicit core(int numThreads, int storeLat, int loadlat) : numThreads(numThreads), storeLat(storeLat), loadLat(loadlat)
    {
        threads.reserve(numThreads);
        for (int i = 0; i < numThreads; i++)
        {
            threads.push_back(std::unique_ptr<thread>(new thread()));
        }
    }
    
    double getCPI()
    {
        return (double)numCyclesTot/(double)numInstrTot;
    }
    
    bool executionCompleted()
    {
        for(auto & threadObj : threads)
        {
            if(!threadObj.get()->isHalted())
                return false;
        }
        
        return true;
    }
    
    tcontext getRegFile(int threadID)
    {
        return threads[threadID].get()->getRegFile();
    }
    
    void executeCycle(int threadID, Instruction instruction)
    {
        //std::cout << "executing: thread " << threadID << " with command " << instruction.opcode << std::endl;
        numInstrTot++;
        numCyclesTot++;
        
        int destReg = instruction.dst_index;
        int src1Reg = instruction.src1_index;
        int src2RegImm = instruction.src2_index_imm;
        bool src2IsImm = instruction.isSrc2Imm;
        switch(instruction.opcode) //for each case update regFile (if needed) and add cycles
        {
            case CMD_NOP:
                break;
            case CMD_ADD:
                threads[threadID].get()->updateMemRegFile(destReg, src1Reg, src2RegImm, CMD_ADD, src2IsImm);
                threads[threadID].get()->setCycles(1);
                break;
            case CMD_SUB:
                threads[threadID].get()->updateMemRegFile(destReg, src1Reg, src2RegImm, CMD_SUB, src2IsImm);
                threads[threadID].get()->setCycles(1);
                break;
            case CMD_ADDI:
                threads[threadID].get()->updateMemRegFile(destReg, src1Reg, src2RegImm, CMD_ADDI, src2IsImm);
                threads[threadID].get()->setCycles(1);
                break;
            case CMD_SUBI:
                threads[threadID].get()->updateMemRegFile(destReg, src1Reg, src2RegImm, CMD_SUBI, src2IsImm);
                threads[threadID].get()->setCycles(1);
                break;
            case CMD_LOAD:
                threads[threadID].get()->updateMemRegFile(destReg, src1Reg, src2RegImm, CMD_LOAD, src2IsImm);
                threads[threadID].get()->setCycles(1+loadLat);
                break;
            case CMD_STORE:
                threads[threadID].get()->updateMemRegFile(destReg, src1Reg, src2RegImm, CMD_STORE, src2IsImm);
                threads[threadID].get()->setCycles(1+storeLat);
                break;
            case CMD_HALT:
                threads[threadID].get()->halt();
                threads[threadID].get()->setCycles(1);
                break;
            default:
                break;
                
        }
    }
    
    void updateAllThreadCycles()
    {
        for(auto& x : threads)
            x.get()->countCycle();
    }
    
    void haltThread(int threadID)
    {
        threads[threadID].get()->halt();
    }
    
    bool noThreadsReady()
    {
        for(auto& x : threads)
        {
            if(x.get()->isReady() && (!x.get()->isHalted()))
                return false;
        }
        
        return true;
    }
    
    void incrementTotalCycles(int cycles)
    {
        numCyclesTot += cycles;
    }
    
    int getNextAvailableThread(int currentID, bool isFg, bool firstExecution)
    {
        for(int i = currentID; i < numThreads; i++)
        {
            if((!threads[i].get()->isHalted()) && threads[i].get()->isReady())
            {
                if(isFg && i == currentID && !firstExecution)
                    continue;
                return i;
            }
        }
        
        for(int i = 0; i < currentID; i++)
        {
            if((!threads[i].get()->isHalted()) && threads[i].get()->isReady())
                return i;
        }
        
        return currentID;
    }
    
    void setCurrentThreadID(int ID)
    {
        currentThread = ID;
    }
    
    int getCurrentThread()
    {
        return currentThread;
    }
    
    int getInstructionLine(int ID)
    {
        return threads[ID].get()->getLineToRead();
    }
    
    void incrementInstructionLine(int ID)
    {
        threads[ID].get()->nextLine();
    }
    
    bool threadIsReady(int ID)
    {
        return threads[ID]->isReady();
    }
    
    int getNumThreads()
    {
        return numThreads;
    }
    
};

static std::unique_ptr<core> blockedCore;
static std::unique_ptr<core> fgCore;

void executeMT(std::unique_ptr<core>& core, bool isBlocked)
{
    Instruction currentInstruction = Instruction(); 
    int instructionLine;
    int threadID = 0;
    bool firstExecution = true;
    while(!core.get()->executionCompleted())
    {
        core.get()->updateAllThreadCycles();
        if(core.get()->noThreadsReady())
        {
            core.get()->incrementTotalCycles(1); //execute IDLE
            continue;
        }
        core.get()->setCurrentThreadID(core.get()->getNextAvailableThread(threadID, !isBlocked, firstExecution));
        if(isBlocked && core.get()->getCurrentThread() != threadID)
            //If threadID has changed execute context switch for blocked MT
        {
            core.get()->incrementTotalCycles(SIM_GetSwitchCycles());
            for(int i = 0; i < SIM_GetSwitchCycles(); i++)
                core.get()->updateAllThreadCycles();
        }
        threadID = core.get()->getCurrentThread();
        instructionLine = core.get()->getInstructionLine(threadID);
        core.get()->incrementInstructionLine(threadID);
        SIM_MemInstRead(instructionLine, &currentInstruction, threadID);
        core.get()->executeCycle(threadID, currentInstruction);
        firstExecution = false;
    }
}

void CORE_BlockedMT()
{
    blockedCore.reset(new core(SIM_GetThreadsNum(), SIM_GetStoreLat(), SIM_GetLoadLat()));
    
    executeMT(blockedCore, true);
}

void CORE_FinegrainedMT()
{
    fgCore.reset(new core(SIM_GetThreadsNum(), SIM_GetStoreLat(), SIM_GetLoadLat()));
    
    executeMT(fgCore, false);
}

double CORE_BlockedMT_CPI()
{
    return blockedCore.get()->getCPI();
}

double CORE_FinegrainedMT_CPI()
{
    return fgCore.get()->getCPI();
}

void CORE_BlockedMT_CTX(tcontext* context, int threadid)
{
    for(int i = 0; i < REGS_COUNT; i++)
    {
        context[threadid].reg[i] = blockedCore.get()->getRegFile(threadid).reg[i];
    }
}

void CORE_FinegrainedMT_CTX(tcontext* context, int threadid)
{
    for(int i = 0; i < REGS_COUNT; i++)
    {
        context[threadid].reg[i] = fgCore.get()->getRegFile(threadid).reg[i];
    }

}


