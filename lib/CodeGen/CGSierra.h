#ifndef LLVM_CLANG_CODEGEN_CGSIERRA_H
#define LLVM_CLANG_CODEGEN_CGSIERRA_H

#include "CGBuilder.h"

namespace llvm {
  class Type;
  class Value;
  class StoreInst;
}

namespace clang {
namespace CodeGen {

class CodeGenFunction;

llvm::Value *EmitSierraConversion(CodeGenFunction &CGF, llvm::Value *Src, QualType SrcType, QualType DstType);

llvm::StoreInst *EmitMaskedStore(CGBuilderTy &Builder, llvm::Value *Mask, 
                                 llvm::Value *Val, llvm::Value *Ptr, bool Volatile);


llvm::Value *Mask1ToMask8(CGBuilderTy &Builder, llvm::Value *Mask1);
llvm::Value *Mask8ToMask1(CGBuilderTy &Builder, llvm::Value *Mask8);

}  // end namespace CodeGen
}  // end namespace clang

#endif
