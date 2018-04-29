//===--- CGSierra.cpp - Code Generation for Sierra ------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//  This file implements Sierra-related code generation.
//
//===----------------------------------------------------------------------===//

#include "clang/AST/Type.h"
#include "CGSierra.h"
#include "CodeGenFunction.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Value.h"
#include "llvm/IR/Intrinsics.h"

using llvm::Value;

namespace clang {
namespace CodeGen {

llvm::Value *EmitSierraConversion(CodeGenFunction &CGF, Value *Src,
                                  QualType SrcType, QualType DstType) {
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
  } else if (isa<llvm::IntegerType>(DstTy)) {
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

llvm::Value *EmitMaskedStore(CGBuilderTy &Builder, llvm::Value *Mask,
                                 llvm::Value *Val, Address Ptr, bool Volatile) {
  llvm::VectorType *MaskTy = llvm::cast<llvm::VectorType>(Mask->getType());
  llvm::VectorType *ValTy  = llvm::cast<llvm::VectorType>( Val->getType());
  assert(MaskTy->getNumElements() == ValTy->getNumElements());
  llvm::Value *OldVal = Builder.CreateLoad(Ptr);
  llvm::Value *NewVal = Builder.CreateSelect(Mask, Val, OldVal);
  return Builder.CreateStore(NewVal, Ptr, Volatile);

}

static llvm::Value *AllFalseInt(llvm::Type *Type) {
  return llvm::ConstantInt::get(Type, uint64_t(0));
}

llvm::Value *EmitVecToInt(CGBuilderTy &Builder, llvm::Value *Vec) {
  llvm::VectorType *VecTy = llvm::cast<llvm::VectorType>(Vec->getType());
  unsigned NumElems = VecTy->getNumElements();
#if 0
  llvm::Module* Module = Builder.GetInsertBlock()->getParent()->getParent();
  llvm::Function *Fun = llvm::Intrinsic::getDeclaration(Module, llvm::Intrinsic::x86_avx_movmsk_ps_256);
  MaskTy = llvm::VectorType::get(Builder.getInt32Ty(), NumElems);
  Mask = Builder.CreateSExt(Mask, MaskTy);
  MaskTy = llvm::VectorType::get(Builder.getFloatTy(), NumElems);
  Mask = Builder.CreateBitCast(Mask, MaskTy);

  return Builder.CreateCall(Fun, Mask);
#else
  llvm::VectorType *Vec8Ty = llvm::VectorType::get(llvm::IntegerType::get(Builder.getContext(), 8), NumElems);
  llvm::Value *SExt = Builder.CreateSExt(Vec, Vec8Ty);
  return Builder.CreateBitCast(SExt, llvm::IntegerType::get(Builder.getContext(), NumElems*8));
#endif
}

llvm::Value *EmitAnyTrue(CodeGenFunction &CGF, llvm::Value *V) {
  CGBuilderTy &Builder = CGF.Builder;
  //auto M = CGF.getSierraMask();
  //assert(M && "no current mask");
  llvm::Value *Result = EmitVecToInt(Builder, V);
  return Builder.CreateICmpNE(Result, AllFalseInt(Result->getType()));
}

llvm::Value *EmitAnyFalse(CodeGenFunction &CGF, llvm::Value *V) {
  CGBuilderTy &Builder = CGF.Builder;
  return EmitAnyTrue(CGF, Builder.CreateNot(V));
}

llvm::Value *EmitAllFalse(CodeGenFunction &CGF, llvm::Value *V) {
  CGBuilderTy &Builder = CGF.Builder;
  auto M = CGF.getSierraMask();
  assert(M && "no current mask");
  llvm::Value *Result = EmitVecToInt(Builder, V);
  return Builder.CreateICmpEQ(Result, AllFalseInt( Result->getType()));
}

llvm::Value *EmitAllTrue(CodeGenFunction &CGF, llvm::Value *V) {
  CGBuilderTy &Builder = CGF.Builder;
  return EmitAllFalse(CGF, Builder.CreateNot(V));
}

//------------------------------------------------------------------------------

llvm::Constant *CreateAllOnesVector(llvm::LLVMContext &Context,
                                    unsigned NumElems) {
  llvm::Constant** ones = new llvm::Constant*[NumElems];
  for (unsigned i = 0; i < NumElems; ++i)
    ones[i] = llvm::ConstantInt::getTrue(Context);

  llvm::ArrayRef<llvm::Constant*> values(ones, NumElems);
  llvm::Constant *result = llvm::ConstantVector::get(values);
  delete[] ones;
  return result;
}

llvm::Constant *CreateAllZerosVector(llvm::LLVMContext &Context,
                                     unsigned NumElems) {
  llvm::Constant** zeros = new llvm::Constant*[NumElems];
  for (unsigned i = 0; i < NumElems; ++i)
    zeros[i] = llvm::ConstantInt::getFalse(Context);

  llvm::ArrayRef<llvm::Constant*> values(zeros, NumElems);
  llvm::Constant *result = llvm::ConstantVector::get(values);
  delete[] zeros;
  return result;
}

//------------------------------------------------------------------------------

void EmitSierraIfStmt(CodeGenFunction &CGF, IfStmt &S) {
  CGBuilderTy &Builder = CGF.Builder;

  auto OldMask = CGF.getSierraMask();
  bool resetToScalar = not OldMask;
  if (resetToScalar) {
    /* Create a mask with all-true current and all-false continue. */
    unsigned NumElems = S.getCond()->IgnoreImpCasts()->getType()->getSierraVectorLength();
    OldMask = SierraMask(Builder.getContext(), NumElems);
    CGF.setSierraMask(OldMask);
  }
  llvm::Type *MaskTy = OldMask.CurrentMask->getType();

  llvm::BasicBlock *ThenBlock = CGF.createBasicBlock("sierra-if.then");
  llvm::BasicBlock *ElseBlock = CGF.createBasicBlock("sierra-if.else");
  llvm::BasicBlock *EndBlock  = CGF.createBasicBlock("sierra-if.end");

  /* The masks at the end of the Then-/Else-Stmt.  We use these to compute the
   * SierraMask for all further stmts. */
  llvm::PHINode *ElseCurrentPhi   = llvm::PHINode::Create(MaskTy, 2);
  llvm::PHINode *ElseContinuePhi  = llvm::PHINode::Create(MaskTy, 2);
  llvm::PHINode *EndCurrentPhi    = llvm::PHINode::Create(MaskTy, 2);
  llvm::PHINode *EndContinuePhi   = llvm::PHINode::Create(MaskTy, 2);

  /* The PHI nodes for the incoming masks. */
  llvm::PHINode *ThenPhi = NULL, *ElsePhi = NULL;

  CGF.EmitBranchOnBoolExpr(S.getCond(),
                           ThenBlock, ElseBlock,
                           0 /*TODO*/,
                           false,
                           &ThenPhi, &ElsePhi);

  /* Emit the then-block. */
  CGF.EmitBlock(ThenBlock);
  {
    CGF.setSierraMask(SierraMask(ThenPhi, OldMask.ContinueMask));

    CodeGenFunction::RunCleanupsScope ThenScope(CGF);
    CGF.EmitStmt(S.getThen());

    /* Add edges to phi-nodes */
    {
      auto M = CGF.getSierraMask();
      llvm::BasicBlock *BB = Builder.GetInsertBlock();
      ElsePhi->addIncoming(ThenPhi, BB);
      ElseCurrentPhi->addIncoming(M.CurrentMask, BB);
      ElseContinuePhi->addIncoming(M.ContinueMask, BB);
      EndCurrentPhi->addIncoming(M.CurrentMask, BB);
      EndContinuePhi->addIncoming(M.ContinueMask, BB);
    }

    CGF.setSierraMask(OldMask);  // necessary for the conditional branch
    Builder.CreateCondBr(
        EmitAnyTrue(CGF, Builder.CreateAnd(OldMask.CurrentMask,
                                           Builder.CreateNot(ThenPhi))),
        ElseBlock, EndBlock);
  }

  /* Emit the else-block. */
  CGF.EmitBlock(ElseBlock);
  {
    /* Add the missing incoming edges for SierraMask. */
    {
      llvm::BasicBlock *BB = ElseCurrentPhi->getIncomingBlock(0);
      assert(BB);

      for (unsigned i = 0, E = ElsePhi->getNumIncomingValues(); i != E; ++i) {
        llvm::BasicBlock *InBB = ElsePhi->getIncomingBlock(i);
        if (InBB != BB) {
          ElseCurrentPhi->addIncoming(OldMask.CurrentMask, InBB);
          ElseContinuePhi->addIncoming(OldMask.ContinueMask, InBB);
        }
      }
    }
    Builder.Insert(ElseCurrentPhi);
    Builder.Insert(ElseContinuePhi);

    CGF.setSierraMask(SierraMask(
          Builder.CreateAnd(Builder.CreateNot(ElsePhi), OldMask.CurrentMask),
          OldMask.ContinueMask));

    if (Stmt *Else = S.getElse()) {
      CodeGenFunction::RunCleanupsScope ElseScope(CGF);
      CGF.EmitStmt(Else);
    }

    {
      auto M = CGF.getSierraMask();
      llvm::BasicBlock *BB = Builder.GetInsertBlock();
      EndCurrentPhi->addIncoming(
          Builder.CreateOr(M.CurrentMask, ElseCurrentPhi), BB);
      EndContinuePhi->addIncoming(
          Builder.CreateOr(M.ContinueMask, ElseContinuePhi), BB);
    }
  }

  CGF.EmitBlock(EndBlock);
  Builder.Insert(EndCurrentPhi);
  Builder.Insert(EndContinuePhi);
  CGF.setSierraMask(resetToScalar ? SierraMask() : SierraMask(EndCurrentPhi, EndContinuePhi));
}

//------------------------------------------------------------------------------

void EmitSierraWhileStmt(CodeGenFunction &CGF, WhileStmt &S) {
  CGBuilderTy &Builder = CGF.Builder;

  auto OldMask = CGF.getSierraMask();
  bool resetToScalar = not OldMask;
  if (resetToScalar) {
    /* Create a mask with all-true current and all-false continue. */
    unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();
    OldMask = SierraMask(Builder.getContext(), NumElems);
    CGF.setSierraMask(OldMask);
  }
  llvm::Type *MaskTy = OldMask.CurrentMask->getType();

  CodeGenFunction::JumpDest Continue = CGF.getJumpDestInCurrentScope("sierra-while.continue");
  CodeGenFunction::JumpDest LoopExit = CGF.getJumpDestInCurrentScope("sierra-while.end");
  llvm::BasicBlock *CondBlock        = CGF.createBasicBlock("sierra-while.cond");
  llvm::BasicBlock *BodyBlock        = CGF.createBasicBlock("sierra-while.body");
  llvm::BasicBlock *ExitBlock        = LoopExit.getBlock();
  llvm::BasicBlock *ContBlock        = Continue.getBlock();

  llvm::PHINode *CondMaskPhi = llvm::PHINode::Create(MaskTy, 0, "sierra-while.phi-cond");
  CondMaskPhi->addIncoming(OldMask.CurrentMask, Builder.GetInsertBlock());

  llvm::PHINode *ContinueMask = llvm::PHINode::Create(
      MaskTy, 0, "sierra-while.continue-mask");
  CGF.SierraContinuePhiStack.push_back(ContinueMask);

  // C++ [stmt.while]p2:
  //   When the condition of a while statement is a declaration, the
  //   scope of the variable that is declared extends from its point
  //   of declaration (3.3.2) to the end of the while statement.
  //   [...]
  //   The object created in a condition is destroyed and created
  //   with each iteration of the loop.

  CGF.EmitBlock(CondBlock);
  Builder.Insert(CondMaskPhi);
  CGF.setSierraMask(SierraMask(CondMaskPhi, OldMask.ContinueMask));

  CodeGenFunction::RunCleanupsScope ConditionScope(CGF);
  if (S.getConditionVariable())
    CGF.EmitAutoVarDecl(*S.getConditionVariable());

  /* If there are any cleanups between here and the loop-exit scope, create a
   * block to stage a loop exit along.
   */
  if (ConditionScope.requiresCleanups())
    ExitBlock = CGF.createBasicBlock("sierra-while.exit");

  llvm::PHINode *LoopMaskPhi = NULL;
  CGF.EmitBranchOnBoolExpr(S.getCond(),
                           BodyBlock, ExitBlock,
                           0 /*TODO*/,
                           false,
                           &LoopMaskPhi);

  /* Emit the loop body.  We have to emit this in a cleanup scope because it
   * might be a singleton DeclStmt.
   */
  CGF.EmitBlock(BodyBlock);
  {
    CodeGenFunction::RunCleanupsScope BodyScope(CGF);

    CGF.SierraLoopMaskStack.push_back(LoopMaskPhi);
    CGF.setSierraMask(SierraMask(LoopMaskPhi,
                                 CreateAllZerosVector(
                                   Builder.getContext(),
                                   MaskTy->getVectorNumElements())));

    CGF.BreakContinueStack.push_back(
        CodeGenFunction::BreakContinue(LoopExit, Continue));
    CGF.EmitStmt(S.getBody());
    CGF.BreakContinueStack.pop_back();

    auto M = CGF.getSierraMask();
    ContinueMask->addIncoming(M.ContinueMask, Builder.GetInsertBlock());
  }

  /* Emit the catch-up block for continue stmts. */
  CGF.EmitBlock(ContBlock);
  Builder.Insert(ContinueMask);

  CondMaskPhi->addIncoming(Builder.CreateOr(
                             CGF.getSierraMask().CurrentMask,
                             ContinueMask),
                           Builder.GetInsertBlock());
  ConditionScope.ForceCleanup();
  CGF.EmitBranch(CondBlock);

  CGF.setSierraMask(OldMask);

  if (ExitBlock != LoopExit.getBlock()) {
    CGF.EmitBlock(ExitBlock);
    CGF.EmitBranchThroughCleanup(LoopExit);
  }

  // Emit the exit block.
  CGF.EmitBlock(LoopExit.getBlock());
  CGF.setSierraMask(resetToScalar ? SierraMask() : OldMask);
}


void EmitSierraDoStmt(CodeGenFunction &CGF, DoStmt &S) {
  CGBuilderTy &Builder = CGF.Builder;

  auto OldMask = CGF.getSierraMask();
  bool resetToScalar = not OldMask;
  if (resetToScalar) {
    /* Create a mask with all-true current and all-false continue. */
    unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();
    OldMask = SierraMask(Builder.getContext(), NumElems);
    CGF.setSierraMask(OldMask);
  }
  llvm::Type *MaskTy = OldMask.CurrentMask->getType();

  CodeGenFunction::JumpDest LoopExit = CGF.getJumpDestInCurrentScope("sierra-do.end");
  CodeGenFunction::JumpDest LoopCond = CGF.getJumpDestInCurrentScope("sierra-do.cond");
  llvm::BasicBlock *LoopBody = CGF.createBasicBlock("sierra-do.body");
  llvm::BasicBlock *ExitBlock = LoopExit.getBlock();
  llvm::BasicBlock *OldBlock  = Builder.GetInsertBlock();
  llvm::BasicBlock *CondBlock = LoopCond.getBlock();

  llvm::PHINode *LoopMaskPhi = llvm::PHINode::Create(MaskTy, 0, "sierra-do.phi-body");
  LoopMaskPhi->addIncoming(OldMask.CurrentMask, OldBlock);

  llvm::PHINode *ContinueMask = llvm::PHINode::Create(
      MaskTy, 0, "sierra-while.continue-mask");
  CGF.SierraContinuePhiStack.push_back(ContinueMask);

  CGF.EmitBlock(LoopBody);
  {
    CodeGenFunction::RunCleanupsScope BodyScope(CGF);

    CGF.SierraLoopMaskStack.push_back(LoopMaskPhi);
    Builder.Insert(LoopMaskPhi);

    CGF.setSierraMask(SierraMask(LoopMaskPhi,
                                 CreateAllZerosVector(
                                   Builder.getContext(),
                                   MaskTy->getVectorNumElements())));
    CGF.BreakContinueStack.push_back(
        CodeGenFunction::BreakContinue(LoopExit, LoopCond));
    CGF.EmitStmt(S.getBody());
    CGF.BreakContinueStack.pop_back();

    auto M = CGF.getSierraMask();
    ContinueMask->addIncoming(M.ContinueMask, Builder.GetInsertBlock());
  }

  // Create the block for the condition.
  CGF.EmitBlock(CondBlock);
  {
    CodeGenFunction::RunCleanupsScope ConditionScope(CGF);

    Builder.Insert(ContinueMask);

    if (ConditionScope.requiresCleanups())
      ExitBlock = CGF.createBasicBlock("sierra-do.exit");

    auto M = CGF.getSierraMask();
    CGF.setSierraMask(SierraMask(
          Builder.CreateOr(M.CurrentMask, ContinueMask),
          CreateAllZerosVector(Builder.getContext(),
                               MaskTy->getVectorNumElements())));

    CGF.EmitBranchOnBoolExpr(S.getCond(),
                             LoopBody, ExitBlock,
                             0 /*TODO*/,
                             false,
                             &LoopMaskPhi);
  }

  if (ExitBlock != LoopExit.getBlock()) {
    CGF.EmitBlock(ExitBlock);
    CGF.EmitBranchThroughCleanup(LoopExit);
  }

  // Emit the blocks after the loop
  CGF.EmitBlock(LoopExit.getBlock());
  CGF.setSierraMask(resetToScalar ? SierraMask() : OldMask);
}

void EmitSierraForStmt(CodeGenFunction &CGF, ForStmt &S) {
  CGBuilderTy &Builder = CGF.Builder;
  unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();

  auto OldMask = CGF.getSierraMask();
  bool resetToScalar = not OldMask;
  if (resetToScalar) {
    /* Create a mask with all-true current and all-false continue. */
    OldMask = SierraMask(Builder.getContext(), NumElems);
    CGF.setSierraMask(OldMask);
  }
  llvm::Type *MaskTy = OldMask.CurrentMask->getType();

  CodeGenFunction::JumpDest LoopExit = CGF.getJumpDestInCurrentScope("sierra-for.end");
  CodeGenFunction::JumpDest LoopIncr = CGF.getJumpDestInCurrentScope("sierra-for.inc");
  llvm::BasicBlock *BodyBlock  = CGF.createBasicBlock("sierra-for.body");
  llvm::BasicBlock *CondBlock = CGF.createBasicBlock("sierra-for.cond");
  llvm::BasicBlock *ExitBlock = LoopExit.getBlock();
  llvm::BasicBlock *IncrBlock = LoopIncr.getBlock();

  if (S.getInit())
    CGF.EmitStmt(S.getInit());

  llvm::PHINode *CondMaskPhi = llvm::PHINode::Create(MaskTy, 0,
                                                     "sierra-for.phi-cond");
  CondMaskPhi->addIncoming(OldMask.CurrentMask, Builder.GetInsertBlock());

  /* Used to collect the incoming edges for the loop's increment block. */
  llvm::PHINode *IncrContMask = llvm::PHINode::Create(
      MaskTy, 0, "sierra-for.continue-mask");
  CGF.SierraContinuePhiStack.push_back(IncrContMask);

  CodeGenFunction::RunCleanupsScope ForScope(CGF);

  CGF.EmitBlock(CondBlock);
  Builder.Insert(CondMaskPhi);
  CGF.setSierraMask(SierraMask(CondMaskPhi, OldMask.ContinueMask));

  CodeGenFunction::RunCleanupsScope ConditionScope(CGF);
  if (S.getConditionVariable())
    CGF.EmitAutoVarDecl(*S.getConditionVariable());

  /* If there are any cleanups between here and the loop-exit scope, create a
   * block to stage a loop exit along.
   */
  if (ConditionScope.requiresCleanups())
    ExitBlock = CGF.createBasicBlock("sierra-for.cond.cleanup");

  llvm::PHINode *LoopMaskPhi = NULL;
  CGF.EmitBranchOnBoolExpr(S.getCond(),
                           BodyBlock, ExitBlock,
                           0 /*TODO*/,
                           false,
                           &LoopMaskPhi);


  /* Emit the loop body.  We have to emit this in a cleanup scope because it
   * might be a singleton DeclStmt.
   */
  CGF.EmitBlock(BodyBlock);
  {
    CGF.SierraLoopMaskStack.push_back(LoopMaskPhi);

    // Create a cleanups scope for the loop body
    CodeGenFunction::RunCleanupsScope BodyScope(CGF);

    /* Set the current mask for the body block and the increment block. */
    CGF.setSierraMask(SierraMask(LoopMaskPhi,
                                 CreateAllZerosVector(
                                   Builder.getContext(),
                                   MaskTy->getVectorNumElements())));

    CGF.BreakContinueStack.push_back(
        CodeGenFunction::BreakContinue(LoopExit, LoopIncr));
    CGF.EmitStmt(S.getBody());
    CGF.BreakContinueStack.pop_back();

    auto M = CGF.getSierraMask();
    IncrContMask->addIncoming(M.ContinueMask, Builder.GetInsertBlock());
  }

  CGF.EmitBlock(IncrBlock);
  {
    Builder.Insert(IncrContMask);

    CGF.setSierraMask(SierraMask(
          Builder.CreateOr(CGF.getSierraMask().CurrentMask, IncrContMask),
          CreateAllZerosVector(Builder.getContext(), NumElems)));

    if (S.getInc())
      CGF.EmitStmt(S.getInc());

    CondMaskPhi->addIncoming(CGF.getSierraMask().CurrentMask,
                             Builder.GetInsertBlock());

    ConditionScope.ForceCleanup();
    CGF.EmitBranch(CondBlock);
  }

  if (ExitBlock != LoopExit.getBlock()) {
    CGF.EmitBlock(ExitBlock);
    CGF.EmitBranchThroughCleanup(LoopExit);
  }

  // Emit the blocks after the loop
  CGF.EmitBlock(LoopExit.getBlock());
  ForScope.ForceCleanup();
  CGF.setSierraMask(resetToScalar ? SierraMask() : OldMask);
}

void EmitSierraBreakStmt(CodeGenFunction &CGF, BreakStmt &S) {
  CGBuilderTy &Builder = CGF.Builder;

  /* Insert the new Current mask to the Sierra mask map. */
  auto OldMask = CGF.getSierraMask();

  CGF.setSierraMask(SierraMask(
        CreateAllZerosVector(
          CGF.getLLVMContext(),
          OldMask.CurrentMask->getType()->getVectorNumElements()),
        OldMask.ContinueMask)); // keep continue-mask

  llvm::BasicBlock *CleanupBB = CGF.createBasicBlock("sierra-break.cleanup");
  llvm::BasicBlock *AfterBB   = CGF.createBasicBlock("sierra-break.after");

  Builder.CreateCondBr(
      EmitAnyTrue(CGF, Builder.CreateAnd(Builder.CreateNot(OldMask.CurrentMask),
                                         CGF.SierraLoopMaskStack.back())),
      AfterBB,
      CleanupBB);

  CGF.EmitBlock(CleanupBB);
  CGF.EmitBranchThroughCleanup(CGF.BreakContinueStack.back().BreakBlock);

  CGF.EmitBlock(AfterBB);
}

void EmitSierraContinueStmt(CodeGenFunction &CGF, ContinueStmt &S) {
  CGBuilderTy &Builder = CGF.Builder;

  /* Insert the new Current and Continue mask to the Sierra mask map. */
  auto OldMask = CGF.getSierraMask();

  CGF.setSierraMask(SierraMask(
        CreateAllZerosVector(
          CGF.getLLVMContext(),
          OldMask.CurrentMask->getType()->getVectorNumElements()),
        Builder.CreateOr(OldMask.CurrentMask, OldMask.ContinueMask)));

  llvm::BasicBlock *CleanupBB = CGF.createBasicBlock("sierra-continue.cleanup");
  llvm::BasicBlock *AfterBB   = CGF.createBasicBlock("sierra-continue.after");

  // TODO emit conditional branch to loop exit
  Builder.CreateCondBr(
      EmitAnyTrue(CGF, Builder.CreateAnd(Builder.CreateNot(OldMask.CurrentMask),
                                         CGF.SierraLoopMaskStack.back())),
      AfterBB,
      CleanupBB);

  CGF.EmitBlock(CleanupBB);
  /* Add a new incoming edge for the phi-node in the continue-/increment-block.
   */
  CGF.SierraContinuePhiStack.back()->addIncoming(
      CGF.getSierraMask().ContinueMask,
      Builder.GetInsertBlock());
  CGF.EmitBranchThroughCleanup(CGF.BreakContinueStack.back().ContinueBlock);

  CGF.EmitBlock(AfterBB);
}

void EmitSierraReturnStmt(CodeGenFunction &CGF, ReturnStmt &S) {
  CGBuilderTy &Builder = CGF.Builder;

  auto OldMask = CGF.getSierraMask();

  llvm::BasicBlock *CleanupBB = CGF.createBasicBlock("sierra-return.cleanup");
  llvm::BasicBlock *AfterBB   = CGF.createBasicBlock("sierra-return.after");

  const Expr *RV = S.getRetValue();

  // Treat block literals in a return expression as if they appeared
  // in their own scope.  This permits a small, easily-implemented
  // exception to our over-conservative rules about not jumping to
  // statements following block literals with non-trivial cleanups.
  CodeGenFunction::RunCleanupsScope cleanupScope(CGF);
  if (const ExprWithCleanups *cleanups = dyn_cast_or_null<ExprWithCleanups>(RV)) {
    CGF.enterFullExpression(cleanups);
    RV = cleanups->getSubExpr();
  }

  EmitMaskedStore(Builder,
                  OldMask.CurrentMask,
                  CGF.EmitScalarExpr(RV),
                  CGF.ReturnValue,
                  false);

  ++CGF.NumReturnExprs;
  if (RV == 0 || RV->isEvaluatable(CGF.getContext()))
    ++CGF.NumSimpleReturnExprs;

  CGF.setSierraMask(SierraMask(
        CreateAllZerosVector(
          Builder.getContext(),
          OldMask.CurrentMask->getType()->getVectorNumElements()),
        OldMask.ContinueMask));

  /* FIXME To support masked function calls (SPMD functions), we have to compare
   * the current mask to the function mask.
   */
  Builder.CreateCondBr(EmitAllTrue(CGF, OldMask.CurrentMask),
                       CleanupBB, AfterBB);

  CGF.EmitBlock(CleanupBB);
  cleanupScope.ForceCleanup();
  CGF.EmitBranchThroughCleanup(CGF.ReturnBlock);

  CGF.EmitBlock(AfterBB);
}

}
}
