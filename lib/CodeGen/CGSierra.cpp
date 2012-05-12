#include "clang/AST/Type.h"
#include "CGSierra.h"
#include "CodeGenFunction.h"
#include "llvm/Type.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Value.h"


using llvm::Value;

namespace clang {
namespace CodeGen {

llvm::Value *EmitSierraConversion(CodeGenFunction &CGF, Value *Src, QualType SrcType, QualType DstType) {
  assert(SrcType->isSierraVectorType() && "must be a sierra vector");

  llvm::Type *SrcTy = CGF.ConvertType(SrcType);
  llvm::Type *DstTy = CGF.ConvertType(DstType);

  assert(SrcTy->isVectorTy() && "must be a vector");
  assert(DstTy->isVectorTy() && "must be a vector");

  QualType SrcE = SrcType->getAs<SierraVectorType>()->getElementType();
  QualType DstE = DstType->getAs<SierraVectorType>()->getElementType();

  llvm::VectorType *SrcV = cast<llvm::VectorType>(SrcTy);
  llvm::VectorType *DstV = cast<llvm::VectorType>(DstTy);

  SrcTy = SrcV->getElementType();
  DstTy = DstV->getElementType();

  Value *Res = NULL;

  // Cast to half via float
  if (DstE->isHalfType()) {
    assert(false && "TODO");
    DstTy = CGF.FloatTy;
  }

  if (isa<llvm::IntegerType>(SrcTy)) {
    bool InputSigned = SrcE->isSignedIntegerOrEnumerationType();
    if (isa<llvm::IntegerType>(DstTy))
      Res = CGF.Builder.CreateIntCast(Src, DstV, InputSigned, "conv");
    else if (InputSigned)
      Res = CGF.Builder.CreateSIToFP(Src, DstV, "conv");
    else
      Res = CGF.Builder.CreateUIToFP(Src, DstV, "conv");
  } else if (isa<llvm::IntegerType>(DstV)) {
    assert(SrcTy->isFloatingPointTy() && "Unknown real conversion");
    if (DstE->isSignedIntegerOrEnumerationType())
      Res = CGF.Builder.CreateFPToSI(Src, DstV, "conv");
    else
      Res = CGF.Builder.CreateFPToUI(Src, DstV, "conv");
  } else {
    assert(SrcTy->isFloatingPointTy() && DstTy->isFloatingPointTy() &&
          "Unknown real conversion");
    if (DstTy->getTypeID() < SrcTy->getTypeID())
      Res = CGF.Builder.CreateFPTrunc(Src, DstV, "conv");
    else
      Res = CGF.Builder.CreateFPExt(Src, DstV, "conv");
  }

  // TODO handle halfs
  //assert(DstTy->isVectorTy() && "must be a vector");

  //if (DstTy != ResTy) {
    //assert(ResTy->isIntegerTy(16) && "Only half FP requires extra conversion");
    //Res = Builder.CreateCall(CGF.CGM.getIntrinsic(llvm::Intrinsic::convert_to_fp16), Res);
  //}

  return Res;
}

//------------------------------------------------------------------------------

llvm::StoreInst *EmitMaskedStore(CGBuilderTy &Builder, llvm::Value *Mask, 
                                 llvm::Value *Val, llvm::Value *Ptr, bool Volatile) {
  llvm::VectorType *VMask = llvm::cast<llvm::VectorType>(Mask->getType());
  llvm::VectorType *VVal  = llvm::cast<llvm::VectorType>( Val->getType());
  assert(VMask->getNumElements() == VVal->getNumElements());
  llvm::Value *OldVal = Builder.CreateLoad(Ptr);
  llvm::Value *NewVal = Builder.CreateSelect(Mask, Val, OldVal);
  return Builder.CreateStore(NewVal, Ptr, Volatile);
}

llvm::Value *EmitMask1ToMask8(CGBuilderTy &Builder, llvm::Value *Mask1) {
  llvm::LLVMContext& Context = Builder.getContext();
  llvm::VectorType *Mask1Ty = llvm::cast<llvm::VectorType>(Mask1->getType());
  assert(Mask1Ty->getElementType()->isIntegerTy(1) && "wrong mask type");
  unsigned NumElems = Mask1Ty->getNumElements();
  llvm::VectorType *Mask8Ty = llvm::VectorType::get(llvm::IntegerType::get(Context, 8), NumElems);
  return Builder.CreateSExt(Mask1, Mask8Ty);
}

llvm::Value *EmitMask8ToMask1(CGBuilderTy &Builder, llvm::Value *Mask8) {
  llvm::LLVMContext& Context = Builder.getContext();
  llvm::VectorType *Mask8Ty = llvm::cast<llvm::VectorType>(Mask8->getType());
  assert(Mask8Ty->getElementType()->isIntegerTy(8) && "wrong mask type");
  unsigned NumElems = Mask8Ty->getNumElements();
  llvm::VectorType *Mask1Ty = llvm::VectorType::get(llvm::IntegerType::get(Context, 1), NumElems);
  return Builder.CreateTrunc(Mask8, Mask1Ty);
}

//------------------------------------------------------------------------------

void EmitSierraIfStmt(CodeGenFunction &CGF, const IfStmt &S) {
  CGBuilderTy &Builder = CGF.Builder;
  llvm::BasicBlock *ThenBlock = CGF.createBasicBlock("vectorized-if.then");
  llvm::BasicBlock *ContBlock = CGF.createBasicBlock("vectorized-if.end");
  llvm::BasicBlock *ElseBlock = ContBlock;
  if (S.getElse())
    ElseBlock = CGF.createBasicBlock("vectorized-if.else");

  llvm::Value* OldMask = CGF.getCurrentMask();
  llvm::Value* Cond = EmitMask8ToMask1(Builder, CGF.EmitScalarExpr(S.getCond()));
  llvm::Value* ThenMask = Builder.CreateAnd(OldMask, Cond);
  llvm::Value* ElseMask;
  if (S.getElse())
    ElseMask = Builder.CreateNot(ThenMask);

  CGF.EmitBlock(ThenBlock); 
  {
    CGF.setCurrentMask(ThenMask);
    CodeGenFunction::RunCleanupsScope ThenScope(CGF);
    CGF.EmitStmt(S.getThen());
    CGF.setCurrentMask(OldMask);
  }
  if (S.getElse())
    CGF.EmitBranch(ElseBlock);
  else
    CGF.EmitBranch(ContBlock);

  // Emit the 'else' code if present.
  if (const Stmt *Else = S.getElse()) {
    CGF.setCurrentMask(ElseMask);
    CGF.EmitBlock(ElseBlock);
    {
      CodeGenFunction::RunCleanupsScope ElseScope(CGF);
      CGF.EmitStmt(Else);
      CGF.setCurrentMask(OldMask);
    }
    CGF.EmitBranch(ContBlock);
  }

  // Emit the continuation block for code after the if.
  CGF.EmitBlock(ContBlock, true);

  return;
}

//------------------------------------------------------------------------------

static llvm::Constant *CreateAllZerosVector(llvm::LLVMContext &Context, unsigned NumElems) {
  llvm::Constant** zeros = new llvm::Constant*[NumElems];
  for (unsigned i = 0; i < NumElems; ++i)
    zeros[i] = llvm::ConstantInt::getFalse(Context);

  llvm::ArrayRef<llvm::Constant*> values(zeros, NumElems);
  llvm::Constant *result = llvm::ConstantVector::get(values);
  delete[] zeros;
  return result;
}

//------------------------------------------------------------------------------

void EmitSierraWhileStmt(CodeGenFunction &CGF, const WhileStmt &S) {
  CGBuilderTy &Builder = CGF.Builder;
  llvm::LLVMContext &Context = Builder.getContext();
  llvm::BasicBlock* OldBlock = Builder.GetInsertBlock();

  // Emit the header for the loop, which will also become
  // the continue target.
  CodeGenFunction::JumpDest LoopHeader = CGF.getJumpDestInCurrentScope("vectorized-while.cond");
  CGF.EmitBlock(LoopHeader.getBlock());
  llvm::PHINode* phi = Builder.CreatePHI(CGF.getCurrentMask()->getType(), 2, "loop-mask");

  // Create an exit block for when the condition fails, which will
  // also become the break target.
  CodeGenFunction::JumpDest LoopExit = CGF.getJumpDestInCurrentScope("vectorized-while.end");

  // TODO this is not working ATM
  // Store the blocks to use for break and continue.
  //CGF.BreakContinueStack.push_back(BreakContinue(LoopExit, LoopHeader));

  // C++ [stmt.while]p2:
  //   When the condition of a while statement is a declaration, the
  //   scope of the variable that is declared extends from its point
  //   of declaration (3.3.2) to the end of the while statement.
  //   [...]
  //   The object created in a condition is destroyed and created
  //   with each iteration of the loop.
  CodeGenFunction::RunCleanupsScope ConditionScope(CGF);

  if (S.getConditionVariable())
    CGF.EmitAutoVarDecl(*S.getConditionVariable());
  
  // As long as at least one lane yields true go to the loop body.
  llvm::BasicBlock *LoopBody = CGF.createBasicBlock("vectorized-while.body");
  llvm::BasicBlock *ExitBlock = LoopExit.getBlock();

  llvm::Value* OldMask = CGF.getCurrentMask();

  llvm::Value *Cond8 = CGF.EmitScalarExpr(S.getCond());
  unsigned NumElems = cast<llvm::VectorType>(Cond8->getType())->getNumElements();
  llvm::Value *CondI = Builder.CreateBitCast(Cond8, llvm::IntegerType::get(Context, NumElems*8));
  llvm::Value *Cond1 = EmitMask8ToMask1(Builder, Cond8);
  llvm::Value *LoopMask = Builder.CreateAnd(phi, Cond1);
  if (ConditionScope.requiresCleanups())
    ExitBlock = CGF.createBasicBlock("vectorized-while.exit");

  phi->addIncoming( OldMask, OldBlock);
  phi->addIncoming(LoopMask, LoopBody);
  CGF.setCurrentMask(phi);

  llvm::Value* ScalarCond = Builder.CreateICmpNE(CondI, 
      llvm::ConstantInt::get(llvm::IntegerType::get(Context, NumElems*8), 0));
  Builder.CreateCondBr(ScalarCond, LoopBody, ExitBlock);

  if (ExitBlock != LoopExit.getBlock()) {
    CGF.EmitBlock(ExitBlock);
    CGF.EmitBranchThroughCleanup(LoopExit);
  }
 
  // Emit the loop body.  We have to emit this in a cleanup scope
  // because it might be a singleton DeclStmt.
  {
    CodeGenFunction::RunCleanupsScope BodyScope(CGF);
    CGF.EmitBlock(LoopBody);
    CGF.EmitStmt(S.getBody());
  }

  //BreakContinueStack.pop_back();

  // Immediately force cleanup.
  ConditionScope.ForceCleanup();

  // Branch to the loop header again.
  CGF.EmitBranch(LoopHeader.getBlock());

  CGF.setCurrentMask(OldMask);

  // Emit the exit block.
  CGF.EmitBlock(LoopExit.getBlock(), true);
}

//------------------------------------------------------------------------------

}  // end namespace CodeGen
}  // end namespace clang

