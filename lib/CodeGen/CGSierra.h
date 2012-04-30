#ifndef LLVM_CLANG_CODEGEN_CGSIERRA_H
#define LLVM_CLANG_CODEGEN_CGSIERRA_H

namespace llvm {
  class Type;
  class Value;
}

namespace clang {
namespace CodeGen {

class CodeGenFunction;

llvm::Value *EmitSierraConversion(CodeGenFunction &CGF, llvm::Value *Src, QualType SrcType, QualType DstType);

}  // end namespace CodeGen
}  // end namespace clang

#endif
