#include "llvm/IR/Constant.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Metadata.h"

#define MEMINSTRUMENT_MD "rv"
#define NOINSTRUMENT_MD "vectorize_function"


namespace {
bool hasNoInstrumentImpl(llvm::MDNode *N) {
  if (!N || N->getNumOperands() < 1) {
    return false;
  }
  if (auto *Str = clang::dyn_cast<llvm::MDString>(N->getOperand(0))) {
    if (Str->getString().equals(NOINSTRUMENT_MD)) {
      return true;
    }
  }
  return false;
}
} // namespace


namespace meminstrument {
void setNoInstrument(llvm::GlobalObject *O) {
  auto &Ctx = O->getContext();
  llvm::MDNode *N =
      llvm::MDNode::get(Ctx, llvm::MDString::get(Ctx, NOINSTRUMENT_MD));
  O->setMetadata(MEMINSTRUMENT_MD, N);
}

void setNoInstrument(llvm::Instruction *I) {
  auto &Ctx = I->getContext();
  llvm::MDNode *N =
      llvm::MDNode::get(Ctx, llvm::MDString::get(Ctx, NOINSTRUMENT_MD));
  I->setMetadata(MEMINSTRUMENT_MD, N);
}

void setNoInstrument(llvm::Constant *C) {
  auto O = clang::cast<llvm::GlobalObject>(C);
  setNoInstrument(O);
}

bool hasNoInstrument(llvm::GlobalObject *O) {
  return hasNoInstrumentImpl(O->getMetadata(MEMINSTRUMENT_MD));
}

bool hasNoInstrument(llvm::Instruction *O) {
  return hasNoInstrumentImpl(O->getMetadata(MEMINSTRUMENT_MD));
}
} // namespace meminstrument
