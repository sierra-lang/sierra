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

static llvm::Value *AllTrueInt(llvm::Type *Type) {
  return llvm::ConstantInt::get(Type, uint64_t(-1));
}

static llvm::Value *AllFalseInt(llvm::Type *Type) {
  return llvm::ConstantInt::get(Type, uint64_t(0));
}

static llvm::Value *EmitToInt(CGBuilderTy &Builder, llvm::Value *Vec) {
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

#if 0
llvm::Value *EmitAllTrue(CodeGenFunction &CGF, llvm::Value *Mask) {
  CGBuilderTy &Builder = CGF.Builder;
  llvm::Value *Int = EmitToInt(Builder, Mask);
  return Builder.CreateICmpEQ(Int, AllTrueInt(Int->getType()));
}

llvm::Value *EmitAllFalse(CodeGenFunction &CGF, llvm::Value *Mask) {
  CGBuilderTy &Builder = CGF.Builder;
  llvm::Value *Int = EmitToInt(Builder, Mask);
  return Builder.CreateICmpEQ(Int, AllFalseInt(Int->getType()));
}

llvm::Value *EmitAnyFalse(CodeGenFunction &CGF, llvm::Value *Mask) {
  CGBuilderTy &Builder = CGF.Builder;
  llvm::Value *Int = EmitToInt(Builder, Mask);
  return Builder.CreateICmpNE(Int, AllTrueInt(Int->getType()));
}

llvm::Value *EmitAnyTrue(CodeGenFunction &CGF, llvm::Value *Mask) {
  CGBuilderTy &Builder = CGF.Builder;
  llvm::Value *Int = EmitToInt(Builder, Mask);
  return Builder.CreateICmpNE(Int, AllFalseInt(Int->getType()));
}
#else
llvm::Value *EmitAnyTrue(CodeGenFunction &CGF, llvm::Value *V)
{
  CGBuilderTy &Builder = CGF.Builder;
  llvm::Value *Result = EmitToInt( Builder,
                                   Builder.CreateAnd( V,
                                                      CGF.getCurrentMask() ) );
  return Builder.CreateICmpNE( Result, AllFalseInt( Result->getType() ) );
}

llvm::Value *EmitAnyFalse(CodeGenFunction &CGF, llvm::Value *V)
{
  CGBuilderTy &Builder = CGF.Builder;
  llvm::Value *Result = EmitToInt( Builder,
                                   Builder.CreateAnd( Builder.CreateNot( V ),
                                                      CGF.getCurrentMask() ) );
  return Builder.CreateICmpNE( Result, AllFalseInt( Result->getType() ) );
}

llvm::Value *EmitAllTrue(CodeGenFunction &CGF, llvm::Value *V)
{
  CGBuilderTy &Builder = CGF.Builder;
  llvm::Value *Result = EmitToInt( Builder,
                                   Builder.CreateAnd( Builder.CreateNot( V ),
                                                      CGF.getCurrentMask() ) );
  return Builder.CreateICmpEQ( Result, AllFalseInt( Result->getType() ) );
}

llvm::Value *EmitAllFalse(CodeGenFunction &CGF, llvm::Value *V)
{
  CGBuilderTy &Builder = CGF.Builder;
  llvm::Value *Result = EmitToInt( Builder,
                                   Builder.CreateAnd( V, CGF.getCurrentMask() ) );
  return Builder.CreateICmpEQ( Result, AllFalseInt( Result->getType() ) );
}
#endif

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

void EmitSierraIfStmt(CodeGenFunction &CGF, const IfStmt &S) {
  CGBuilderTy &Builder = CGF.Builder;
  llvm::LLVMContext &Context = Builder.getContext();

  unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();
  assert(NumElems > 1);
  bool noCurrentMask = false;
  if (!CGF.getCurrentMask()) {
    noCurrentMask = true;
    CGF.setCurrentMask(CreateAllOnesVector(Context, NumElems));
  }

  llvm::BasicBlock *ThenBlock = CGF.createBasicBlock("sierra-if.then");
  llvm::BasicBlock *ContBlock = CGF.createBasicBlock("sierra-if.end");
  llvm::BasicBlock *ElseBlock = CGF.createBasicBlock("sierra-if.else");

  llvm::Value* OldMask = CGF.getCurrentMask();

  llvm::PHINode *ThenPhi = NULL, *ElsePhi = NULL;

  CGF.EmitBranchOnBoolExpr( S.getCond(), ThenBlock, ElseBlock, false, &ThenPhi,
                            &ElsePhi );

  CGF.EmitBlock(ThenBlock);
  {
    CGF.setCurrentMask( Builder.CreateAnd( ThenPhi, OldMask ) );
    CodeGenFunction::RunCleanupsScope ThenScope(CGF);
    CGF.EmitStmt(S.getThen());
    ElsePhi->addIncoming( ThenPhi, Builder.GetInsertBlock() );
    CGF.setCurrentMask(OldMask);
    Builder.CreateCondBr( EmitAllTrue(CGF, ThenPhi), ContBlock, ElseBlock );
  }

  // Emit the 'else' code if present.
	CGF.EmitBlock(ElseBlock);
	{
		if ( const Stmt *Else = S.getElse() )
		{
			CGF.setCurrentMask( Builder.CreateAnd( OldMask,
																						 Builder.CreateNot( ElsePhi ) ) );

			CodeGenFunction::RunCleanupsScope ElseScope(CGF);
			CGF.EmitStmt(Else);
			CGF.setCurrentMask(OldMask);
		}
	}

  if (noCurrentMask)
    CGF.setCurrentMask(0);
  else
    CGF.setCurrentMask(OldMask);

  CGF.EmitBlock( ContBlock );
}

//------------------------------------------------------------------------------

void EmitSierraWhileStmt(CodeGenFunction &CGF, const WhileStmt &S) {
  CGBuilderTy &Builder = CGF.Builder;
  llvm::LLVMContext &Context = Builder.getContext();
  llvm::Value *OldMask = CGF.getCurrentMask();
  unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();
  llvm::VectorType *MaskTy = llvm::VectorType::get(
      llvm::IntegerType::getInt1Ty( Context ), NumElems );

  assert(NumElems > 1);
  if (!OldMask)
    CGF.setCurrentMask(CreateAllOnesVector(Context, NumElems));

  /* Create a new basic block for the loop condition. */
  llvm::BasicBlock *CondBlock = CGF.createBasicBlock( "sierra-while.cond" );
  /* Create a special continue target.  Its block is placed after the loop body.
   * All continue stmts will branch to that block.
   */
  CodeGenFunction::JumpDest Continue =
    CGF.getJumpDestInCurrentScope( "sierra-while.continue" );
  CodeGenFunction::JumpDest LoopExit =
    CGF.getJumpDestInCurrentScope( "sierra-while.end" );

  // Create a new phi node. The arguments are specified later.
  // The phi node will take either the initial loop mask,
  // or the one that is created after evaluating the condition.
  llvm::PHINode *phi = llvm::PHINode::Create( MaskTy, 0,
                                              "sierra-while.phi-header" );
  phi->addIncoming( CGF.getCurrentMask(), Builder.GetInsertBlock() );

  // C++ [stmt.while]p2:
  //   When the condition of a while statement is a declaration, the
  //   scope of the variable that is declared extends from its point
  //   of declaration (3.3.2) to the end of the while statement.
  //   [...]
  //   The object created in a condition is destroyed and created
  //   with each iteration of the loop.
  CodeGenFunction::RunCleanupsScope ConditionScope(CGF);

  CGF.EmitBlock( CondBlock );
  Builder.Insert( phi );
  CGF.setCurrentMask( phi );

  if (S.getConditionVariable())
    CGF.EmitAutoVarDecl(*S.getConditionVariable());

  // As long as at least one lane yields true go to the loop body.
  llvm::BasicBlock *LoopBody = CGF.createBasicBlock( "sierra-while.body" );
  llvm::BasicBlock *ExitBlock = LoopExit.getBlock();

  // If there are any cleanups between here and the loop-exit scope,
  // create a block to stage a loop exit along.
  if (ConditionScope.requiresCleanups())
    ExitBlock = CGF.createBasicBlock("sierra-while.exit");

  llvm::PHINode *LoopMaskPhi = NULL;
  CGF.EmitBranchOnBoolExpr( S.getCond(),
                            LoopBody, ExitBlock,
                            false, &LoopMaskPhi );

  // Emit the loop body.  We have to emit this in a cleanup scope
  // because it might be a singleton DeclStmt.
  llvm::Value *LoopMask;
  {
    CodeGenFunction::RunCleanupsScope BodyScope(CGF);
    CGF.EmitBlock(LoopBody);

    LoopMask = Builder.CreateAnd( LoopMaskPhi, CGF.getCurrentMask() );
    CGF.setCurrentMask(LoopMask);

    CGF.BreakContinueStack.push_back(
        CodeGenFunction::BreakContinue( LoopExit, Continue ) );
    CGF.EmitStmt(S.getBody());
    CGF.BreakContinueStack.pop_back();
  }

  /* Emit the catch-up block for continue stmts. */
  CGF.EmitBlock( Continue.getBlock() );
  phi->addIncoming( LoopMask, Builder.GetInsertBlock() );
  ConditionScope.ForceCleanup();
  CGF.EmitBranch( CondBlock );
  CGF.setCurrentMask(OldMask);

  if ( ExitBlock != LoopExit.getBlock() )
  {
    CGF.EmitBlock(ExitBlock);
    CGF.EmitBranchThroughCleanup(LoopExit);
  }

  // Emit the exit block.
  CGF.EmitBlock(LoopExit.getBlock(), true);
}


void EmitSierraDoStmt( CodeGenFunction &CGF, const DoStmt &S )
{
  CGBuilderTy &Builder = CGF.Builder;
  llvm::LLVMContext &Context = Builder.getContext();
  llvm::Value *OldMask = CGF.getCurrentMask();
  unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();
  llvm::VectorType *MaskTy = llvm::VectorType::get(
      llvm::IntegerType::getInt1Ty( Context ), NumElems );

  assert( NumElems > 1 );
  if ( ! OldMask )
    CGF.setCurrentMask( CreateAllOnesVector( Context, NumElems ) );

  llvm::Value *CurrentMask = CGF.getCurrentMask();

  /* Break target */
  CodeGenFunction::JumpDest LoopExit = CGF.getJumpDestInCurrentScope( "sierra-do.end" );
  /* Continue target */
  CodeGenFunction::JumpDest LoopCond = CGF.getJumpDestInCurrentScope( "sierra-do.cond" );
  llvm::BasicBlock *LoopBody = CGF.createBasicBlock( "sierra-do.body" );
  llvm::BasicBlock *ExitBlock = LoopExit.getBlock();

  llvm::PHINode* LoopMaskPhi = llvm::PHINode::Create( MaskTy, 0,
                                                   "sierra-do.phi-body" );
  LoopMaskPhi->addIncoming( CurrentMask, Builder.GetInsertBlock() );
  {
    CodeGenFunction::RunCleanupsScope BodyScope( CGF );
    CGF.EmitBlock( LoopBody );
    Builder.Insert( LoopMaskPhi );
    CGF.setCurrentMask( Builder.CreateAnd( LoopMaskPhi, CGF.getCurrentMask() ) );
    CGF.BreakContinueStack.push_back(
        CodeGenFunction::BreakContinue( LoopExit, LoopCond ) );
    CGF.EmitStmt( S.getBody() );
    CGF.BreakContinueStack.pop_back();
  }

  // Create the block for the condition.
  {
    CodeGenFunction::RunCleanupsScope ConditionScope( CGF );
    CGF.EmitBlock( LoopCond.getBlock() );

    if( ConditionScope.requiresCleanups() )
      ExitBlock = CGF.createBasicBlock("sierra-do.exit");

    CGF.EmitBranchOnBoolExpr( S.getCond(), LoopBody, ExitBlock, false,
        &LoopMaskPhi );
  }

  CGF.setCurrentMask( OldMask );

  if ( ExitBlock != LoopExit.getBlock() )
  {
    CGF.EmitBlock( ExitBlock );
    CGF.EmitBranchThroughCleanup( LoopExit );
  }

  // Emit the blocks after the loop
  CGF.EmitBlock( LoopExit.getBlock(), true );
}

void EmitSierraForStmt(CodeGenFunction &CGF, const ForStmt &S)
{
  CGBuilderTy &Builder = CGF.Builder;
  llvm::LLVMContext &Context = Builder.getContext();

  llvm::Value *OldMask = CGF.getCurrentMask();
  unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();
  llvm::VectorType *MaskTy = llvm::VectorType::get(
      llvm::IntegerType::getInt1Ty( Context ), NumElems );

  /* If no current mask exist, create one with all lanes set to active. */
  assert( NumElems > 1 );
  if ( ! OldMask )
    CGF.setCurrentMask( CreateAllOnesVector( Context, NumElems ) );

  CodeGenFunction::RunCleanupsScope ForScope( CGF );

  CodeGenFunction::JumpDest LoopExit = CGF.getJumpDestInCurrentScope( "sierra-for.end" );
  CodeGenFunction::JumpDest Continue = CGF.getJumpDestInCurrentScope( "sierra-for.cond" );
  llvm::BasicBlock *LoopBody = CGF.createBasicBlock( "sierra-for.body" );
  llvm::BasicBlock *CondBlock = Continue.getBlock();
  llvm::BasicBlock *ExitBlock = LoopExit.getBlock();
  if ( S.getInc() )
    Continue = CGF.getJumpDestInCurrentScope( "sierra-for.inc" );

  // The phi node will take either the initial loop mask,
  // or the one that is created after evaluating the condition.
  llvm::PHINode *phi = llvm::PHINode::Create( MaskTy, 0,
                                              "sierra-for.phi-header" );

  /* Evaluate the init expr/decl before the condition block.  It can be appended
   * to the current basic block.
   */
  if ( S.getInit() )
    CGF.EmitStmt( S.getInit() );
  phi->addIncoming( CGF.getCurrentMask(), Builder.GetInsertBlock() );

  /* Emit the Condition Block. */
  CGF.EmitBlock( CondBlock );

  /* Set the mask for the condition block to the incoming phi vlaue. */
  Builder.Insert( phi );
  CGF.setCurrentMask( phi );

  /* Create a cleanups scope for the condition variables. */
  CodeGenFunction::RunCleanupsScope ConditionScope( CGF );

  /* If the for statement has a condition scope, emit the local variable
   * declaration.
   */
  if ( S.getConditionVariable() )
    CGF.EmitAutoVarDecl( *S.getConditionVariable() );

  // If there are any cleanups between here and the loop-exit scope,
  // create a block to stage a loop exit along.
  if ( ConditionScope.requiresCleanups() )
    ExitBlock = CGF.createBasicBlock( "sierra-for.cond.cleanup" );

  /* LoopMask will hold a reference to the llvm::Value representing the loop's
   * mask.
   */
  llvm::PHINode *LoopMaskPhi = NULL;
  CGF.EmitBranchOnBoolExpr( S.getCond(),
                            LoopBody, ExitBlock,
                            false, &LoopMaskPhi );


  // Emit the loop body.  We have to emit this in a cleanup scope
  // because it might be a singleton DeclStmt.
  llvm::Value *LoopMask;
  {
    // Create a cleanups scope for the loop body
    CodeGenFunction::RunCleanupsScope BodyScope( CGF );
    CGF.EmitBlock( LoopBody );

    /* Set the current mask for the body block and the increment block. */
    LoopMask = Builder.CreateAnd( CGF.getCurrentMask(), LoopMaskPhi );
    CGF.setCurrentMask( LoopMask );

    CGF.BreakContinueStack.push_back(
        CodeGenFunction::BreakContinue( LoopExit, Continue ) );
    CGF.EmitStmt( S.getBody() );
    CGF.BreakContinueStack.pop_back();
  }

  // If an increment block exists, make it the new continue target.
  if ( S.getInc() )
  {
    // Create the increment block
    CGF.EmitBlock( Continue.getBlock() );
    CGF.EmitStmt( S.getInc() );
  }

  /* The current basic block is either the Body Block or the Increment Block. */
  phi->addIncoming( LoopMask, Builder.GetInsertBlock() );
  ConditionScope.ForceCleanup();
  CGF.EmitBranch( CondBlock );
  CGF.setCurrentMask( OldMask );

  ForScope.ForceCleanup();

  if ( ExitBlock != LoopExit.getBlock() )
  {
    CGF.EmitBlock(ExitBlock);
    CGF.EmitBranchThroughCleanup(LoopExit);
  }

  // Emit the blocks after the loop
  CGF.EmitBlock( LoopExit.getBlock(), true );
}

//------------------------------------------------------------------------------

}  // end namespace CodeGen
}  // end namespace clang
