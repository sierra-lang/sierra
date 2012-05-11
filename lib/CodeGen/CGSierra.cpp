#include "clang/AST/Type.h"
#include "CGSierra.h"
#include "CodeGenFunction.h"
#include "llvm/Type.h"
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
  //llvm::LLVMContext& Context = Builder.getContext();
  llvm::VectorType *VMask = llvm::cast<llvm::VectorType>(Mask->getType());
  llvm::VectorType *VVal  = llvm::cast<llvm::VectorType>( Val->getType());
  assert(VMask->getNumElements() == VVal->getNumElements());
  //assert(VMask->getElementType()->isIntegerTy(8) && "wrong mask type");
  //unsigned NumElems = VMask->getNumElements();
  //llvm::VectorType *BoolMaskTy = llvm::VectorType::get(llvm::IntegerType::get(Context, 1), NumElems);
  //llvm::Value *BoolMask =  Builder.CreateTrunc(Mask, BoolMaskTy);
  llvm::Value *BoolMask =  Mask;
  llvm::Value *OldVal = Builder.CreateLoad(Ptr);
  llvm::Value *NewVal = Builder.CreateSelect(BoolMask, Val, OldVal);
  return Builder.CreateStore(NewVal, Ptr, Volatile);
}

}  // end namespace CodeGen
}  // end namespace clang

