#include "llvm/IR/Constant.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Metadata.h"

#define RVCGINSTRUMENT_MD "rvcg"
#define VECINSTRUMENT_MD "vectorize_function"

#define OPTINSTRUMENT_MD "rvcg_opt"
#define MASKINSTRUMENT_MD "mask"


namespace {
bool hasVecInstrumentImpl(llvm::MDNode *N) {
  if (!N || N->getNumOperands() < 1) {
    return false;
  }
  if (auto *Str = llvm::dyn_cast<llvm::MDString>(N->getOperand(0))) {
    if (Str->getString().equals(VECINSTRUMENT_MD)) {
      return true;
    }
  }
  return false;
}

bool hasMaskInstrumentImpl(llvm::MDNode *N) {
  if (!N || N->getNumOperands() < 1) {
    return false;
  }
  if (auto *Str = llvm::dyn_cast<llvm::MDString>(N->getOperand(0))) {
    if (Str->getString().equals(MASKINSTRUMENT_MD)) {
      return true;
    }
  }
  return false;
}
} // namespace


namespace rvcginstrument {
void setVecInstrument(llvm::GlobalObject *O) {
  auto &Ctx = O->getContext();
  llvm::MDNode *N =
      llvm::MDNode::get(Ctx, llvm::MDString::get(Ctx, VECINSTRUMENT_MD));
  O->setMetadata(RVCGINSTRUMENT_MD, N);
}

void setMaskInstrument(llvm::GlobalObject *O) {
  auto &Ctx = O->getContext();
  llvm::MDNode *N =
      llvm::MDNode::get(Ctx, llvm::MDString::get(Ctx, MASKINSTRUMENT_MD));
  O->setMetadata(OPTINSTRUMENT_MD, N);
}

bool hasVecInstrument(llvm::GlobalObject *O) {
  return hasVecInstrumentImpl(O->getMetadata(RVCGINSTRUMENT_MD));
}

bool hasMaskInstrument(llvm::GlobalObject *O) {
  return hasMaskInstrumentImpl(O->getMetadata(OPTINSTRUMENT_MD));
}

//void setVecInstrument(llvm::Instruction *I) {
  //auto &Ctx = I->getContext();
  //llvm::MDNode *N =
      //llvm::MDNode::get(Ctx, llvm::MDString::get(Ctx, VECINSTRUMENT_MD));
  //I->setMetadata(RVCGINSTRUMENT_MD, N);
//}

//void setVecInstrument(llvm::Constant *C) {
  //auto O = llvm::cast<llvm::GlobalObject>(C);
  //setVecInstrument(O);
//}

//bool hasVecInstrument(llvm::Instruction *O) {
  //return hasVecInstrumentImpl(O->getMetadata(RVCGINSTRUMENT_MD));
//}
} // namespace rvcginstrument
