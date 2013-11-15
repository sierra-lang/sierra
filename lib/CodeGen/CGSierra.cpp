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

static llvm::Value *AllTrueInt(CGBuilderTy &Builder, llvm::Type *Type) {
  return llvm::ConstantInt::get(Type, uint64_t(-1));
}

static llvm::Value *AllFalseInt(CGBuilderTy &Builder, llvm::Type *Type) {
  return llvm::ConstantInt::get(Type, uint64_t(0));
}

static llvm::Value *EmitToInt(CGBuilderTy &Builder, llvm::Value *Mask) {
  llvm::VectorType *MaskTy = llvm::cast<llvm::VectorType>(Mask->getType());
  unsigned NumElems = MaskTy->getNumElements();
  //return Builder.CreateBitCast(Mask, llvm::IntegerType::get(Builder.getContext(), NumElems));

  llvm::VectorType *Mask8Ty = llvm::VectorType::get(llvm::IntegerType::get(Builder.getContext(), 8), NumElems);
  llvm::Value *SExt = Builder.CreateSExt(Mask, Mask8Ty);
  return Builder.CreateBitCast(SExt, llvm::IntegerType::get(Builder.getContext(), NumElems*8));
}

llvm::Value *EmitAllTrue(CGBuilderTy &Builder, llvm::Value *Mask) {
  llvm::Value *Int = EmitToInt(Builder, Mask);
  return Builder.CreateICmpEQ(Int, AllTrueInt(Builder, Int->getType()));
}

llvm::Value *EmitAllFalse(CGBuilderTy &Builder, llvm::Value *Mask) {
  llvm::Value *Int = EmitToInt(Builder, Mask);
  return Builder.CreateICmpEQ(Int, AllFalseInt(Builder, Int->getType()));
}

llvm::Value *EmitAnyFalse(CGBuilderTy &Builder, llvm::Value *Mask) {
  llvm::Value *Int = EmitToInt(Builder, Mask);
  return Builder.CreateICmpNE(Int, AllTrueInt(Builder, Int->getType()));
}

llvm::Value *EmitAnyTrue(CGBuilderTy &Builder, llvm::Value *Mask) {
  llvm::Value *Int = EmitToInt(Builder, Mask);
  return Builder.CreateICmpNE(Int, AllFalseInt(Builder, Int->getType()));
}

//------------------------------------------------------------------------------

#if 0

static llvm::Constant *CreateAllOnesVectorPTest(llvm::LLVMContext &Context, unsigned NumElems) {
  llvm::Constant** ones = new llvm::Constant*[NumElems];
  for (unsigned i = 0; i < NumElems; ++i)
    ones[i] = llvm::ConstantInt::getIntegerValue(
      llvm::IntegerType::get(Context, 64), 
      llvm::APInt::getAllOnesValue(64));

  llvm::ArrayRef<llvm::Constant*> values(ones, NumElems);
  llvm::Constant *result = llvm::ConstantVector::get(values);
  delete[] ones;
  return result;
}

static llvm::Constant *CreateAllZerosVector(llvm::LLVMContext &Context, unsigned NumElems) {
  llvm::Constant** zeros = new llvm::Constant*[NumElems];
  for (unsigned i = 0; i < NumElems; ++i)
    zeros[i] = llvm::ConstantInt::getFalse(Context);

  llvm::ArrayRef<llvm::Constant*> values(zeros, NumElems);
  llvm::Constant *result = llvm::ConstantVector::get(values);
  delete[] zeros;
  return result;
}

#endif

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

  llvm::BasicBlock *ThenBlock = CGF.createBasicBlock("vectorized-if.then");
  llvm::BasicBlock *ContBlock = CGF.createBasicBlock("vectorized-if.end");
  llvm::BasicBlock *ElseBlock = CGF.createBasicBlock("vectorized-if.else");

  llvm::Value* OldMask = CGF.getCurrentMask();

  llvm::PHINode *ThenMask, *ElseMask;

  CGF.EmitBranchOnBoolExpr( S.getCond(), ThenBlock, ElseBlock, false, &ThenMask,
                            &ElseMask );

  CGF.EmitBlock(ThenBlock); 
  {
    CGF.setCurrentMask(ThenMask);
    CodeGenFunction::RunCleanupsScope ThenScope(CGF);
    CGF.EmitStmt(S.getThen());
    ElseMask->addIncoming( ThenMask, Builder.GetInsertBlock() );

    CGF.setCurrentMask(OldMask);

    /*
     * Check whether the ThenMask is the current mask
     */
    llvm::Value *CondXor = Builder.CreateXor( ThenMask, CGF.getCurrentMask() );
    llvm::Value *Cond8 = EmitMask1ToMask8( Builder, CondXor );
    llvm::Value *CondI = Builder.CreateBitCast(Cond8, llvm::IntegerType::get(
        Context, NumElems*8));
    llvm::Value *ScalarCond = Builder.CreateICmpEQ(
      CondI, llvm::ConstantInt::get(llvm::IntegerType::get( Context,
                                                            NumElems*8), 0));

		/*
		 * Emit the branch from the Then block to the Else block
		 */
		Builder.CreateCondBr( ScalarCond, ContBlock, ElseBlock );
  }

  // Emit the 'else' code if present.
	CGF.EmitBlock(ElseBlock);
	{
		if ( const Stmt *Else = S.getElse() )
		{
			CGF.setCurrentMask( Builder.CreateAnd( CGF.getCurrentMask(),
																						 Builder.CreateNot( ElseMask ) ) );

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
  llvm::BasicBlock *OldBlock = Builder.GetInsertBlock();
  unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();

  assert(NumElems > 1);
  if (!OldMask)
    CGF.setCurrentMask(CreateAllOnesVector(Context, NumElems));

  llvm::Value *CurrenMask = CGF.getCurrentMask();

  // Emit the header for the loop, which will also become
  // the continue target.
  CodeGenFunction::JumpDest LoopHeader =
    CGF.getJumpDestInCurrentScope("vectorized-while.cond");
  CGF.EmitBlock(LoopHeader.getBlock());

  llvm::PHINode *phi = Builder.CreatePHI( CurrenMask->getType(), 2,
      "vectorized-while.header-phi" );

  phi->addIncoming( CurrenMask, OldBlock );

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

  llvm::PHINode *LoopMask;

  CGF.EmitBranchOnBoolExpr( S.getCond(), LoopBody, ExitBlock, false,
      &LoopMask );

  // Emit the loop body.  We have to emit this in a cleanup scope
  // because it might be a singleton DeclStmt.
  {
    CodeGenFunction::RunCleanupsScope BodyScope(CGF);
    CGF.setCurrentMask(LoopMask);
    CGF.EmitBlock(LoopBody);
    CGF.EmitStmt(S.getBody());
  }

  phi->addIncoming( LoopMask, Builder.GetInsertBlock() );

  //BreakContinueStack.pop_back();

  // Immediately force cleanup.
  ConditionScope.ForceCleanup();

  // Branch to the loop header again.
  CGF.EmitBranch(LoopHeader.getBlock());

  CGF.setCurrentMask(OldMask);

  if (ConditionScope.requiresCleanups())
  {
    ExitBlock = CGF.createBasicBlock("vectorized-while.exit");
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
  llvm::BasicBlock* OldBlock = Builder.GetInsertBlock();

  unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();
  assert( NumElems > 1 );
  bool noCurrentMask = false;
  if ( !CGF.getCurrentMask() )
  {
      noCurrentMask = true;
      CGF.setCurrentMask( CreateAllOnesVector( Context, NumElems ) );
  }
  llvm::Value *OldMask = CGF.getCurrentMask();

  // Break target
  CodeGenFunction::JumpDest LoopExit = CGF.getJumpDestInCurrentScope( "vectorized-do.end" );
  // Continue target
  CodeGenFunction::JumpDest LoopCond = CGF.getJumpDestInCurrentScope( "vectorized-do.cond" );
  // Body block, containing the loop body
  llvm::BasicBlock *LoopBody = CGF.createBasicBlock( "vectorized-do.body" );
  // Exit block ( after the Do stmt )
  llvm::BasicBlock *ExitBlock = LoopExit.getBlock();

/*
  // Store the blocks to use for break and continue.
  BreakContinueStack.push_back(BreakContinue(LoopExit, LoopCond));
*/
  
  // Create a basic block for the loop body.
  CGF.EmitBlock( LoopBody );

  // Get the type for the phi node
  llvm::VectorType* MaskTy = llvm::VectorType::get(llvm::IntegerType::getInt1Ty(Context), NumElems);  

  // Create a new phi node. The arguments are specified later.                                        
  // The phi node will take either the initial loop mask,                                             
  // or the one that is created after evaluating the condition.                                       
  llvm::PHINode* phi = Builder.CreatePHI(MaskTy, 2, "loop-mask");                                     
  
  // Create a new cleanups scope for the body,                                                        
  // so declarations do not interfere with the condition.                                             
  CodeGenFunction::RunCleanupsScope BodyScope( CGF );                                                 

  // Emit the body.                                                                                   
  CGF.EmitStmt( S.getBody() );

  // Create the block for the condition.
  CGF.EmitBlock( LoopCond.getBlock() );
  CodeGenFunction::RunCleanupsScope ConditionScope( CGF );

  phi->addIncoming( OldMask, OldBlock );
  
  // Create the new loop mask.
  llvm::Value *Cond = CGF.EmitScalarExpr( S.getCond() );
  llvm::Value *LoopMask = Builder.CreateAnd( phi, Cond );
  llvm::Value *Cond8 = EmitMask1ToMask8( Builder, LoopMask );
  llvm::Value *CondI = Builder.CreateBitCast( Cond8, llvm::IntegerType::get( Context, NumElems * 8 ) );

  // Current insert block is the last block of the condition part.
  phi->addIncoming( LoopMask, Builder.GetInsertBlock() );

  if( ConditionScope.requiresCleanups() )
    ExitBlock = CGF.createBasicBlock("vectorized-do.exit");

  CGF.setCurrentMask( LoopMask );

  llvm::Value *ScalarCond = Builder.CreateICmpNE( CondI,
    llvm::ConstantInt::get ( llvm::IntegerType::get( Context, NumElems * 8 ), 0 ) );
  Builder.CreateCondBr( ScalarCond, LoopBody, ExitBlock );

  // Emit the exit block
  if ( ExitBlock != LoopExit.getBlock() )
  {
    CGF.EmitBlock( ExitBlock );
    CGF.EmitBranchThroughCleanup( LoopExit );
  }

/*
  BreakContinueStack.pop_back();
*/

  if ( noCurrentMask )
    CGF.setCurrentMask( 0 );
  else
    CGF.setCurrentMask( OldMask );

  // Emit the blocks after the loop
  CGF.EmitBlock( LoopExit.getBlock(), true );
}

void EmitSierraForStmt(CodeGenFunction &CGF, const ForStmt &S)
{
  CGBuilderTy &Builder = CGF.Builder;
  llvm::LLVMContext &Context = Builder.getContext();
  llvm::BasicBlock* OldBlock = Builder.GetInsertBlock();

  unsigned NumElems = S.getCond()->getType()->getSierraVectorLength();
  assert( NumElems > 1 );
  bool noCurrentMask = false;
  if ( !CGF.getCurrentMask() )
  {
      noCurrentMask = true;
      CGF.setCurrentMask( CreateAllOnesVector( Context, NumElems ) );
  }
  llvm::Value *OldMask = CGF.getCurrentMask();

  // Q: Is this for the variables, that are declared inside the statement?
  CodeGenFunction::RunCleanupsScope ForScope( CGF );

  // Break target
  CodeGenFunction::JumpDest LoopExit = CGF.getJumpDestInCurrentScope( "vectorized-for.end" );
  // Continue target
  // Use the Condition Block as the default continue target.
  // If the statement has an increment block, that block will become the new Continue target.
  CodeGenFunction::JumpDest Continue = CGF.getJumpDestInCurrentScope( "vectorized-for.cond" );
  // Body Block, containing the Loop Body
  llvm::BasicBlock *LoopBody = CGF.createBasicBlock( "vectorized-for.body" );
  // Condition Block
  llvm::BasicBlock *CondBlock = Continue.getBlock();
  // Exit Block (after the For Statement)
  llvm::BasicBlock *ExitBlock = LoopExit.getBlock();
  
  // Create a cleanups scope for the condition variables.
  CodeGenFunction::RunCleanupsScope ConditionScope( CGF );

  // Evaluate the first part before the loop. This is the initializing statement of the for-statement.
  // This statement can be appended to the current basic block (OldBlock) .
  if ( S.getInit() )
    CGF.EmitStmt( S.getInit() );

  // Emit the Condition Block
  CGF.EmitBlock( CondBlock );

  // If the for statement has a condition scope, emit the local variable
  // declaration.
  if ( S.getConditionVariable() )
    CGF.EmitAutoVarDecl( *S.getConditionVariable() );

  // Get the type for the phi node
  llvm::VectorType* MaskTy = llvm::VectorType::get(llvm::IntegerType::getInt1Ty(Context), NumElems);

  // Create a new phi node. The arguments are specified later.                                        
  // The phi node will take either the initial loop mask,                                             
  // or the one that is created after evaluating the condition.                                       
  llvm::PHINode* phi = Builder.CreatePHI(MaskTy, 2, "loop-mask");

  // Create the new loop mask.
  llvm::Value *Cond = CGF.EmitScalarExpr( S.getCond() );
  llvm::Value *LoopMask = Builder.CreateAnd( phi, Cond );
  llvm::Value *Cond8 = EmitMask1ToMask8( Builder, LoopMask );
  llvm::Value *CondI = Builder.CreateBitCast( Cond8, llvm::IntegerType::get( Context, NumElems * 8 ) );
  phi->addIncoming( OldMask, OldBlock );

  // If there are any cleanups between here and the loop-exit scope,
  // create a block to stage a loop exit along.
  if ( ConditionScope.requiresCleanups() )
    ExitBlock = CGF.createBasicBlock( "vectorized-for.cond.cleanup" );

  CGF.setCurrentMask( LoopMask );

  llvm::Value *ScalarCond = Builder.CreateICmpNE( CondI,
    llvm::ConstantInt::get ( llvm::IntegerType::get( Context, NumElems * 8 ), 0 ) );
  Builder.CreateCondBr( ScalarCond, LoopBody, ExitBlock );

  // Create the body block
  CGF.EmitBlock( LoopBody );

  {
    // Create a cleanups scope for the loop body
    CodeGenFunction::RunCleanupsScope BodyScope( CGF );
                                                        
    // Emit the loop body
    CGF.EmitStmt( S.getBody() );
  }

  // If an increment block exists, make it the new continue target.
  if ( S.getInc() )
  {
    // Make the increment block the new continue target
    Continue = CGF.getJumpDestInCurrentScope( "vectorized-for.inc" );
    // Create the increment block
    CGF.EmitBlock( Continue.getBlock() );
    CGF.EmitStmt( S.getInc() );
  }

  // Add incoming edge from the current basic block with the loop mask
  // The current basic block is either the Body Block or the Increment Block
  phi->addIncoming( LoopMask, Builder.GetInsertBlock() );

  ConditionScope.ForceCleanup();
  CGF.EmitBranch( CondBlock );

  // Emit the exit block
  if ( ExitBlock != LoopExit.getBlock() )
  {
    CGF.EmitBlock( ExitBlock );
    CGF.EmitBranchThroughCleanup( LoopExit );
  }

  if ( noCurrentMask )
    CGF.setCurrentMask( 0 );
  else
    CGF.setCurrentMask( OldMask );

  // Emit the blocks after the loop
  CGF.EmitBlock( LoopExit.getBlock(), true );
}

//------------------------------------------------------------------------------

}  // end namespace CodeGen
}  // end namespace clang

