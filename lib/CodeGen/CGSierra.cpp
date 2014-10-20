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
#include "llvm/Type.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Value.h"
#include "llvm/Intrinsics.h"

using llvm::Value;

namespace clang {
namespace CodeGen {

/// Selects the Sierra mask for the current basic block from the two given
/// incoming Sierra masks.  Creates PHI nodes if necessary.
SierraMask * EmitSierraSelectMask( CodeGenFunction &CGF,
                                   llvm::BasicBlock *BB0,
                                   SierraMask *M0,
                                   llvm::BasicBlock *BB1,
                                   SierraMask *M1)
{
  assert( M0 && "M0 was null" );
  assert( M1 && "M1 was null" );

  if ( M0 == M1 ) return M0;

  CGBuilderTy &Builder = CGF.Builder;
  SierraMask *M = CGF.getSierraMask();
  llvm::Value *Current, *Continue;

  /* Select Current mask. */
  if ( M0->CurrentMask != M1->CurrentMask )
  {
    llvm::PHINode *phi = Builder.CreatePHI( M->CurrentMask->getType(), 2 );
    phi->addIncoming( M0->CurrentMask, BB0 );
    phi->addIncoming( M1->CurrentMask, BB1 );
    Current = phi;
  }
  else
    Current = M0->CurrentMask;

  /* Select Continue mask. */
  if ( M0->ContinueMask != M1->ContinueMask )
  {
    llvm::PHINode *phi = Builder.CreatePHI( M->ContinueMask->getType(), 2 );
    phi->addIncoming( M0->ContinueMask, BB0 );
    phi->addIncoming( M1->ContinueMask, BB1 );
    Continue = phi;
  }
  else
    Continue = M0->ContinueMask;

  return SierraMask::Create( Current, Continue );
}

/// Merges two Sierra masks.
/// The current masks are merged via AND, the continue masks are merged via OR.
SierraMask * EmitSierraMergeMask( CodeGenFunction &CGF,
                                  SierraMask *M0,
                                  SierraMask *M1 )
{
  assert( M0 && "M0 was null" );
  assert( M1 && "M1 was null" );

  CGBuilderTy &Builder = CGF.Builder;

  llvm::Value *Current, *Continue;

  if ( M0->CurrentMask != M1->CurrentMask )
    Current = Builder.CreateAnd(
        M0->CurrentMask,
        M1->CurrentMask
        );
  else
    Current = M0->CurrentMask;

  if ( M0->ContinueMask != M1->ContinueMask )
    Continue = Builder.CreateOr(
        M0->ContinueMask,
        M1->ContinueMask
        );
  else
    Continue = M0->ContinueMask;

  return SierraMask::Create( Current, Continue );
}

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
                                 llvm::Value *Val, llvm::Value *Ptr, bool Volatile) {
  llvm::VectorType *MaskTy = llvm::cast<llvm::VectorType>(Mask->getType());
  llvm::VectorType *ValTy  = llvm::cast<llvm::VectorType>( Val->getType());
  assert(MaskTy->getNumElements() == ValTy->getNumElements());
  llvm::Value *OldVal = Builder.CreateLoad(Ptr);

#if 0
  llvm::Module* Module = Builder.GetInsertBlock()->getParent()->getParent();
  llvm::Function *Fun = llvm::Intrinsic::getDeclaration(Module, llvm::Intrinsic::x86_avx_blendv_ps_256);
  unsigned NumElems = MaskTy->getNumElements();
  MaskTy = llvm::VectorType::get(Builder.getInt32Ty(), NumElems);
  Mask   = Builder.CreateSExt(Mask, MaskTy);
  MaskTy = llvm::VectorType::get(Builder.getFloatTy(), NumElems);
  Mask   = Builder.CreateBitCast(Mask, MaskTy);
  llvm::Value *NewVal;
  if (ValTy->getElementType()->isFloatingPointTy()) {
    NewVal = Builder.CreateCall3(Fun, OldVal, Val, Mask);
  } else {
    llvm::Type* NewValTy  = llvm::VectorType::get(Builder.getFloatTy(), NumElems);
    Val    = Builder.CreateBitCast(Val, NewValTy);
    OldVal = Builder.CreateBitCast(OldVal, NewValTy);
    NewVal = Builder.CreateCall3(Fun, OldVal, Val, Mask);
    NewVal = Builder.CreateBitCast(NewVal, ValTy);
  }
#else
  llvm::Value *NewVal = Builder.CreateSelect(Mask, Val, OldVal);
#endif
  return Builder.CreateStore(NewVal, Ptr, Volatile);

}

// TODO unused
#if 0
static llvm::Value *AllTrueInt(llvm::Type *Type) {
  return llvm::ConstantInt::get(Type, uint64_t(-1));
}
#endif

static llvm::Value *AllFalseInt(llvm::Type *Type) {
  return llvm::ConstantInt::get(Type, uint64_t(0));
}

llvm::Value *EmitToInt(CGBuilderTy &Builder, llvm::Value *Vec) {
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

llvm::Value *EmitAnyTrue(CodeGenFunction &CGF, llvm::Value *V)
{
  CGBuilderTy &Builder = CGF.Builder;
  SierraMask *M = CGF.getSierraMask();
  assert( M->CurrentMask && "no current mask" );
  llvm::Value *Result = EmitToInt( Builder, V );
  return Builder.CreateICmpNE( Result, AllFalseInt( Result->getType() ) );
}

llvm::Value *EmitAnyFalse(CodeGenFunction &CGF, llvm::Value *V)
{
  CGBuilderTy &Builder = CGF.Builder;
  return EmitAnyTrue( CGF, Builder.CreateNot( V ) );
}

llvm::Value *EmitAllFalse(CodeGenFunction &CGF, llvm::Value *V)
{
  CGBuilderTy &Builder = CGF.Builder;
  SierraMask *M = CGF.getSierraMask();
  assert( M->CurrentMask && "no current mask" );
  llvm::Value *Result = EmitToInt( Builder, V );
  return Builder.CreateICmpEQ( Result, AllFalseInt( Result->getType() ) );
}

llvm::Value *EmitAllTrue(CodeGenFunction &CGF, llvm::Value *V)
{
  CGBuilderTy &Builder = CGF.Builder;
  return EmitAllFalse( CGF, Builder.CreateNot( V ) );
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

void EmitSierraIfStmt(CodeGenFunction &CGF, const IfStmt &S)
{
  CGBuilderTy &Builder = CGF.Builder;

  SierraMask *OldMask = CGF.getSierraMask();
  if ( ! OldMask )
  {
    /* Create a mask with all-true current and all-false continue. */
    unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();
    OldMask = SierraMask::Create( Builder.getContext(), NumElems );
    CGF.setSierraMask( OldMask );
  }
  llvm::Type *MaskTy = OldMask->CurrentMask->getType();

  llvm::BasicBlock *ThenBlock = CGF.createBasicBlock( "sierra-if.then" );
  llvm::BasicBlock *ElseBlock = CGF.createBasicBlock( "sierra-if.else" );
  llvm::BasicBlock *EndBlock  = CGF.createBasicBlock( "sierra-if.end" );

  /* The masks at the end of the Then-/Else-Stmt.  We use these to compute the
   * SierraMask for all further stmts.
   */
  llvm::PHINode *ElseCurrentPhi   = llvm::PHINode::Create( MaskTy, 2 );
  llvm::PHINode *ElseContinuePhi  = llvm::PHINode::Create( MaskTy, 2 );
  llvm::PHINode *EndCurrentPhi    = llvm::PHINode::Create( MaskTy, 2 );
  llvm::PHINode *EndContinuePhi   = llvm::PHINode::Create( MaskTy, 2 );

  /* The PHI nodes for the incoming masks. */
  llvm::PHINode *ThenPhi = NULL, *ElsePhi = NULL;

  CGF.EmitBranchOnBoolExpr( S.getCond(),
                            ThenBlock, ElseBlock, 0 /*TODO*/,
                            false,
                            &ThenPhi, &ElsePhi );

  /* Emit the then-block. */
  CGF.EmitBlock( ThenBlock );
  {
    CGF.setSierraMask( SierraMask::Create( ThenPhi, OldMask->ContinueMask ) );

    CodeGenFunction::RunCleanupsScope ThenScope(CGF);
    CGF.EmitStmt( S.getThen() );

    /* Add edges to phi-nodes */
    {
      SierraMask *M = CGF.getSierraMask();
      llvm::BasicBlock *BB = Builder.GetInsertBlock();
      ElsePhi->addIncoming( ThenPhi, BB );
      ElseCurrentPhi->addIncoming( M->CurrentMask, BB );
      ElseContinuePhi->addIncoming( M->ContinueMask, BB );
      EndCurrentPhi->addIncoming( M->CurrentMask, BB );
      EndContinuePhi->addIncoming( M->ContinueMask, BB );
    }

    CGF.setSierraMask( OldMask );  // necessary for the conditional branch
    Builder.CreateCondBr(
        EmitAnyTrue( CGF, Builder.CreateAnd( OldMask->CurrentMask,
                                             Builder.CreateNot( ThenPhi ) ) ),
        ElseBlock, EndBlock );
  }

  /* Emit the else-block. */
  CGF.EmitBlock( ElseBlock );
  {
    /* Add the missing incoming edges for SierraMask. */
    {
      llvm::BasicBlock *BB = ElseCurrentPhi->getIncomingBlock( 0 );
      assert( BB );

      for ( unsigned i = 0, E = ElsePhi->getNumIncomingValues(); i != E; ++i )
      {
        llvm::BasicBlock *InBB = ElsePhi->getIncomingBlock( i );
        if ( InBB != BB )
        {
          ElseCurrentPhi->addIncoming( OldMask->CurrentMask, InBB );
          ElseContinuePhi->addIncoming( OldMask->ContinueMask, InBB );
        }
      }
    }
    Builder.Insert( ElseCurrentPhi );
    Builder.Insert( ElseContinuePhi );

    CGF.setSierraMask( SierraMask::Create(
          Builder.CreateAnd( Builder.CreateNot( ElsePhi ),
                             OldMask->CurrentMask ),
          OldMask->ContinueMask ) );

    if ( const Stmt *Else = S.getElse() )
    {
      CodeGenFunction::RunCleanupsScope ElseScope( CGF );
      CGF.EmitStmt( Else );
    }

    {
      SierraMask *M = CGF.getSierraMask();
      llvm::BasicBlock *BB = Builder.GetInsertBlock();
      EndCurrentPhi->addIncoming(
          Builder.CreateOr( M->CurrentMask, ElseCurrentPhi ), BB );
      EndContinuePhi->addIncoming(
          Builder.CreateOr( M->ContinueMask, ElseContinuePhi ), BB );
    }
  }

  CGF.EmitBlock( EndBlock );
  Builder.Insert( EndCurrentPhi );
  Builder.Insert( EndContinuePhi );
  CGF.setSierraMask( SierraMask::Create( EndCurrentPhi, EndContinuePhi ) );
}

//------------------------------------------------------------------------------

void EmitSierraWhileStmt(CodeGenFunction &CGF, const WhileStmt &S)
{
  CGBuilderTy &Builder = CGF.Builder;

  SierraMask *OldMask = CGF.getSierraMask();
  if ( ! OldMask )
  {
    /* Create a mask with all-true current and all-false continue. */
    unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();
    OldMask = SierraMask::Create( Builder.getContext(), NumElems );
    CGF.setSierraMask( OldMask );
  }
  llvm::Type *MaskTy = OldMask->CurrentMask->getType();

  CodeGenFunction::JumpDest Continue = CGF.getJumpDestInCurrentScope( "sierra-while.continue" );
  CodeGenFunction::JumpDest LoopExit = CGF.getJumpDestInCurrentScope( "sierra-while.end" );
  llvm::BasicBlock *CondBlock        = CGF.createBasicBlock( "sierra-while.cond" );
  llvm::BasicBlock *LoopBody         = CGF.createBasicBlock( "sierra-while.body" );
  llvm::BasicBlock *ExitBlock        = LoopExit.getBlock();

  llvm::PHINode *CondMaskPhi = llvm::PHINode::Create( MaskTy, 0,
                                                      "sierra-while.phi-cond" );
  CondMaskPhi->addIncoming( OldMask->CurrentMask, Builder.GetInsertBlock() );

  // C++ [stmt.while]p2:
  //   When the condition of a while statement is a declaration, the
  //   scope of the variable that is declared extends from its point
  //   of declaration (3.3.2) to the end of the while statement.
  //   [...]
  //   The object created in a condition is destroyed and created
  //   with each iteration of the loop.

  CGF.EmitBlock( CondBlock );
  Builder.Insert( CondMaskPhi );
  CGF.setSierraMask( SierraMask::Create( CondMaskPhi, OldMask->ContinueMask ) );

  CodeGenFunction::RunCleanupsScope ConditionScope(CGF);
  if (S.getConditionVariable())
    CGF.EmitAutoVarDecl(*S.getConditionVariable());

  /* If there are any cleanups between here and the loop-exit scope, create a
   * block to stage a loop exit along.
   */
  if ( ConditionScope.requiresCleanups() )
    ExitBlock = CGF.createBasicBlock( "sierra-while.exit" );

  llvm::PHINode *LoopMaskPhi = NULL;
  CGF.EmitBranchOnBoolExpr( S.getCond(),
                            LoopBody, ExitBlock, 0 /*TODO*/,
                            false,
                            &LoopMaskPhi );

  /* Emit the loop body.  We have to emit this in a cleanup scope because it
   * might be a singleton DeclStmt.
   */
  {
    CodeGenFunction::RunCleanupsScope BodyScope(CGF);
    CGF.EmitBlock( LoopBody );

    CGF.setSierraMask(
        SierraMask::Create( LoopMaskPhi,
                            CreateAllZerosVector(
                              Builder.getContext(),
                              MaskTy->getVectorNumElements() ) ) );

    CGF.BreakContinueStack.push_back(
        CodeGenFunction::BreakContinue( LoopExit, Continue ) );
    CGF.EmitStmt(S.getBody());
    CGF.BreakContinueStack.pop_back();
  }

  /* Emit the catch-up block for continue stmts. */
  CGF.EmitBlock( Continue.getBlock() );
  CondMaskPhi->addIncoming( Builder.CreateOr(
                              CGF.getSierraMask()->CurrentMask,
                              CGF.getSierraMask()->ContinueMask ),
                            Builder.GetInsertBlock() );
  ConditionScope.ForceCleanup();
  CGF.EmitBranch( CondBlock );

  CGF.setSierraMask( OldMask );

  if ( ExitBlock != LoopExit.getBlock() )
  {
    CGF.EmitBlock( ExitBlock );
    CGF.EmitBranchThroughCleanup( LoopExit );
  }

  // Emit the exit block.
  CGF.EmitBlock( LoopExit.getBlock() );
  CGF.setSierraMask( OldMask );
}


void EmitSierraDoStmt( CodeGenFunction &CGF, const DoStmt &S )
{
  CGBuilderTy &Builder = CGF.Builder;

  SierraMask *OldMask = CGF.getSierraMask();
  if ( ! OldMask )
  {
    /* Create a mask with all-true current and all-false continue. */
    unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();
    OldMask = SierraMask::Create( Builder.getContext(), NumElems );
    CGF.setSierraMask( OldMask );
  }
  llvm::Type *MaskTy = OldMask->CurrentMask->getType();

  CodeGenFunction::JumpDest LoopExit = CGF.getJumpDestInCurrentScope( "sierra-do.end" );
  CodeGenFunction::JumpDest LoopCond = CGF.getJumpDestInCurrentScope( "sierra-do.cond" );
  llvm::BasicBlock *LoopBody = CGF.createBasicBlock( "sierra-do.body" );
  llvm::BasicBlock *ExitBlock = LoopExit.getBlock();
  llvm::BasicBlock *OldBlock = Builder.GetInsertBlock();

  llvm::PHINode *LoopMaskPhi = llvm::PHINode::Create( MaskTy, 0,
                                                      "sierra-do.phi-body" );
  LoopMaskPhi->addIncoming( OldMask->CurrentMask, OldBlock );

  {
    CodeGenFunction::RunCleanupsScope BodyScope( CGF );
    CGF.EmitBlock( LoopBody );

    Builder.Insert( LoopMaskPhi );

    CGF.setSierraMask(
        SierraMask::Create( LoopMaskPhi,
                            CreateAllZerosVector(
                              Builder.getContext(),
                              MaskTy->getVectorNumElements() ) ) );
    CGF.BreakContinueStack.push_back(
        CodeGenFunction::BreakContinue( LoopExit, LoopCond ) );
    CGF.EmitStmt( S.getBody() );
    CGF.BreakContinueStack.pop_back();
  }

  // Create the block for the condition.
  {
    CodeGenFunction::RunCleanupsScope ConditionScope( CGF );
    CGF.EmitBlock( LoopCond.getBlock() );

    if ( ConditionScope.requiresCleanups() )
      ExitBlock = CGF.createBasicBlock("sierra-do.exit");

    SierraMask *M = CGF.getSierraMask();
    CGF.setSierraMask( SierraMask::Create(
          Builder.CreateOr( M->CurrentMask, M->ContinueMask ),
          M->ContinueMask ) );

    CGF.EmitBranchOnBoolExpr( S.getCond(),
                              LoopBody, ExitBlock, 0 /*TODO*/,
                              false,
                              &LoopMaskPhi );
  }

  if ( ExitBlock != LoopExit.getBlock() )
  {
    CGF.EmitBlock( ExitBlock );
    CGF.EmitBranchThroughCleanup( LoopExit );
  }

  // Emit the blocks after the loop
  CGF.EmitBlock( LoopExit.getBlock() );
  CGF.setSierraMask( OldMask );
}

void EmitSierraForStmt(CodeGenFunction &CGF, const ForStmt &S)
{
  CGBuilderTy &Builder = CGF.Builder;
  unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();

  SierraMask *OldMask = CGF.getSierraMask();
  if ( ! OldMask )
  {
    /* Create a mask with all-true current and all-false continue. */
    OldMask = SierraMask::Create( Builder.getContext(), NumElems );
    CGF.setSierraMask( OldMask );
  }
  llvm::Type *MaskTy = OldMask->CurrentMask->getType();

  CodeGenFunction::JumpDest LoopExit = CGF.getJumpDestInCurrentScope( "sierra-for.end" );
  CodeGenFunction::JumpDest LoopIncr = CGF.getJumpDestInCurrentScope( "sierra-for.inc" );
  llvm::BasicBlock *BodyBlock  = CGF.createBasicBlock( "sierra-for.body" );
  llvm::BasicBlock *CondBlock = CGF.createBasicBlock( "sierra-for.cond" );
  llvm::BasicBlock *ExitBlock = LoopExit.getBlock();

  if ( S.getInit() )
    CGF.EmitStmt( S.getInit() );

  llvm::PHINode *CondMaskPhi = llvm::PHINode::Create( MaskTy, 0,
                                                      "sierra-for.phi-cond" );
  CondMaskPhi->addIncoming( OldMask->CurrentMask, Builder.GetInsertBlock() );

  CodeGenFunction::RunCleanupsScope ForScope( CGF );

  CGF.EmitBlock( CondBlock );
  Builder.Insert( CondMaskPhi );
  CGF.setSierraMask( SierraMask::Create( CondMaskPhi, OldMask->ContinueMask ) );

  CodeGenFunction::RunCleanupsScope ConditionScope( CGF );
  if ( S.getConditionVariable() )
    CGF.EmitAutoVarDecl( *S.getConditionVariable() );

  /* If there are any cleanups between here and the loop-exit scope, create a
   * block to stage a loop exit along.
   */
  if ( ConditionScope.requiresCleanups() )
    ExitBlock = CGF.createBasicBlock( "sierra-for.cond.cleanup" );

  llvm::PHINode *LoopMaskPhi = NULL;
  CGF.EmitBranchOnBoolExpr( S.getCond(),
                            BodyBlock, ExitBlock, 0 /*TODO*/,
                            false,
                            &LoopMaskPhi );


  /* Emit the loop body.  We have to emit this in a cleanup scope because it
   * might be a singleton DeclStmt.
   */
  CGF.EmitBlock( BodyBlock );
  {
    // Create a cleanups scope for the loop body
    CodeGenFunction::RunCleanupsScope BodyScope( CGF );

    /* Set the current mask for the body block and the increment block. */
    CGF.setSierraMask(
        SierraMask::Create( LoopMaskPhi,
                            CreateAllZerosVector(
                              Builder.getContext(),
                              MaskTy->getVectorNumElements() ) ) );

    CGF.BreakContinueStack.push_back(
        CodeGenFunction::BreakContinue( LoopExit, LoopIncr ) );
    CGF.EmitStmt( S.getBody() );
    CGF.BreakContinueStack.pop_back();

    SierraMask *M = CGF.getSierraMask();
    IncrCurrMask->addIncoming( M->CurrentMask,  Builder.GetInsertBlock() );
    IncrContMask->addIncoming( M->ContinueMask, Builder.GetInsertBlock() );
  }

  CGF.EmitBlock( LoopIncr.getBlock() );
  {
    Builder.Insert( IncrCurrMask );
    Builder.Insert( IncrContMask );

    CGF.setSierraMask( SierraMask::Create(
          Builder.CreateOr( IncrCurrMask, IncrContMask ),
          CreateAllZerosVector( Builder.getContext(), NumElems )
          ) );

    if ( S.getInc() )
      CGF.EmitStmt( S.getInc() );

    CondMaskPhi->addIncoming( CGF.getSierraMask()->CurrentMask,
                              Builder.GetInsertBlock() );

    ConditionScope.ForceCleanup();
    CGF.EmitBranch( CondBlock );
  }

  if ( ExitBlock != LoopExit.getBlock() )
  {
    CGF.EmitBlock( ExitBlock );
    CGF.EmitBranchThroughCleanup( LoopExit );
  }

  // Emit the blocks after the loop
  CGF.EmitBlock( LoopExit.getBlock() );
  ForScope.ForceCleanup();
  CGF.setSierraMask( OldMask );
}

void EmitSierraBreakStmt(CodeGenFunction &CGF, const BreakStmt &S)
{
  /* Insert the new Current mask to the Sierra mask map. */
  SierraMask *OldMask = CGF.getSierraMask();

  CGF.setSierraMask( SierraMask::Create(
        CreateAllZerosVector(
          CGF.getLLVMContext(),
          OldMask->CurrentMask->getType()->getVectorNumElements() ),
        OldMask->ContinueMask ) ); // keep continue-mask

  // TODO emit conditional branch to loop exit
}

void EmitSierraContinueStmt(CodeGenFunction &CGF, const ContinueStmt &S)
{
  CGBuilderTy &Builder = CGF.Builder;

  /* Insert the new Current and Continue mask to the Sierra mask map. */
  SierraMask *OldMask = CGF.getSierraMask();

  CGF.setSierraMask( SierraMask::Create(
        CreateAllZerosVector(
          CGF.getLLVMContext(),
          OldMask->CurrentMask->getType()->getVectorNumElements() ),
        Builder.CreateOr( OldMask->CurrentMask, OldMask->ContinueMask ) ) );

  // TODO emit conditional branch to continue target
}


//------------------------------------------------------------------------------

}  // end namespace CodeGen
}  // end namespace clang
