#include "clang/Sema/Sema.h"
#include "clang/Sema/Scope.h"
#include "clang/Sema/SemaSierra.h"
#include "clang/AST/OperationKinds.h"
#include "clang/Sema/SemaDiagnostic.h"

namespace clang {

/// \brief Build an sierra vector type.
///
/// Run the required checks for the sierra vector type.
QualType BuildSierraVectorType(Sema &S, QualType T, Expr *ArraySize,
                                     SourceLocation AttrLoc) {
  // FIXME Sierra: actually we want to allow that!!!
  // unlike gcc's vector_size attribute, we do not allow vectors to be defined
  // in conjunction with complex types (pointers, arrays, functions, etc.).
  if (!T->isDependentType() &&
      !T->isIntegerType() && !T->isRealFloatingType()) {
    S.Diag(AttrLoc, diag::err_attribute_invalid_vector_type) << T;
    return QualType();
  }

  if (!ArraySize->isTypeDependent() && !ArraySize->isValueDependent()) {
    llvm::APSInt vecSize(32);
    if (!ArraySize->isIntegerConstantExpr(vecSize, S.Context)) {
      S.Diag(AttrLoc, diag::err_attribute_argument_not_int)
        << "sierra_vector" << ArraySize->getSourceRange();
      return QualType();
    }

    // unlike gcc's vector_size attribute, the size is specified as the
    // number of elements, not the number of bytes.
    unsigned VecS = static_cast<unsigned>(vecSize.getZExtValue());

    if (VecS == 0) {
      S.Diag(AttrLoc, diag::err_attribute_zero_size)
      << ArraySize->getSourceRange();
      return QualType();
    }

    // uniform special case
    if (VecS == 1)
      return QualType();

    unsigned CurS = S.getCurScope()->getCurrentVectorLength();
    // TODO polymorphism
    if (CurS != 1 && CurS != VecS) {
      S.Diag(AttrLoc, diag::err_incompatible_vector_lengths_in_decl)
        << CurS << VecS;
      return QualType();
    }

    QualType res = S.Context.getSierraVectorType(T, VecS);
    return res;
  }

  return S.Context.getDependentSizedSierraVectorType(T, ArraySize, AttrLoc);
}

class SierraVectorOperandsChecker {
public:
  SierraVectorOperandsChecker(Sema &S, ExprResult &LHS, ExprResult &RHS, 
                              SourceLocation Loc, bool IsCompAssign);
  ~SierraVectorOperandsChecker();

  QualType Check();
  QualType Error();
  QualType Splat();
  QualType RightFromLeft(CastKind CK);
  QualType LeftFromRight(CastKind CK);

private:

  Sema &S;
  ExprResult &LHS;
  ExprResult &RHS;
  SourceLocation Loc;
  bool IsCompAssign;
  bool Swapped;
  QualType LHSType;
  QualType RHSType;
  const SierraVectorType* LV;
  const SierraVectorType* RV;
  QualType LE;
  QualType RE;
  unsigned LNum;
  unsigned RNum;
};

SierraVectorOperandsChecker::SierraVectorOperandsChecker(Sema &S, 
                                                         ExprResult &LHS, 
                                                         ExprResult &RHS, 
                                                         SourceLocation Loc, 
                                                         bool IsCompAssign) 
  : S(S), LHS(LHS), RHS(RHS), Loc(Loc), 
    IsCompAssign(IsCompAssign) {

  LHSType = S.Context.getCanonicalType(LHS.get()->getType()).getUnqualifiedType();
  RHSType = S.Context.getCanonicalType(RHS.get()->getType()).getUnqualifiedType();

  // Normalize to have the RHS always be a sierra vector
  if (RHSType->isSierraVectorType() && !IsCompAssign) {
    Swapped = true;
    std::swap(RHS, LHS);
    std::swap(RHSType, LHSType);
  } else
    Swapped = false;

  assert((!IsCompAssign || !Swapped) && "an compassign expr may not be swapped");

  LV      = LHSType->getAs<SierraVectorType>();
  RV      = RHSType->getAs<SierraVectorType>();
  LE      =      LV->getElementType();
  RE      = RV ? RV->getElementType() : RHSType;
  LNum    = LV->getNumElements();
  RNum    = RV ? RV->getNumElements() : 0;
}

SierraVectorOperandsChecker::~SierraVectorOperandsChecker() {
    if (Swapped) std::swap(RHS, LHS);
}

QualType SierraVectorOperandsChecker::Error() {
  QualType res = S.InvalidOperands(Loc, LHS, RHS);
  return res;
}

QualType SierraVectorOperandsChecker::Splat() {
  if (LE == RE) {
    if (RNum) {
      assert(LNum == RNum && "vector lengths do not match");
      return S.Context.getSierraVectorType(LE, RNum);
    }

    QualType NewRType = S.Context.getSierraVectorType(LE, LNum);
    RHS = S.ImpCastExprToType(RHS.take(), NewRType, CK_VectorSplat);
    return NewRType;
  }

  return Error();
}

QualType SierraVectorOperandsChecker::RightFromLeft(CastKind CK) {
  RE = LE;
  QualType NewRType = RNum ? S.Context.getSierraVectorType(RE, RNum) : RE;
  RHS = S.ImpCastExprToType(RHS.take(), NewRType, CK);
  return Splat();
}

QualType SierraVectorOperandsChecker::LeftFromRight(CastKind CK) {
  if (!IsCompAssign) {
    LE = RE;
    QualType NewLType = S.Context.getSierraVectorType(LE, LNum);
    LHS = S.ImpCastExprToType(LHS.take(), NewLType, CK);
  }
  return Splat();
}

QualType SierraVectorOperandsChecker::Check() {
  if (!LHSType->isSierraVectorType() && !LHSType->isScalarType()) return Error();

  if (RNum && LNum != RNum) return Error();

  // Apply unary and bitfield promotions to the LE's type.
  QualType LEUnpromotedType = LE;
  if (LE->isPromotableIntegerType())
    LE = S.Context.getPromotedIntegerType(LE);
  QualType LEBitfieldPromoteTy = S.Context.isPromotableBitField(LHS.get());
  if (!LEBitfieldPromoteTy.isNull())
    LE = LEBitfieldPromoteTy;

  // LE is the promoted LHS vector element type
  if (LE != LEUnpromotedType && !IsCompAssign) {
    LHS = S.ImpCastExprToType(LHS.take(), 
                            S.Context.getSierraVectorType(LE, LNum), 
                            CK_IntegralCast);
  }

  bool IsLFloat = LE->isRealFloatingType();
  bool IsRFloat = RE->isRealFloatingType();

  if (LE == RE) return Splat();

  // LE and RE are floating
  if (IsLFloat && IsRFloat) {
    int order = S.Context.getFloatingTypeOrder(LE, RE);
    if (order > 0) return RightFromLeft(CK_FloatingCast);

    assert(order < 0 && "illegal float comparison");
    return LeftFromRight(CK_FloatingCast);
  } 

  // just LE is floating
  if (IsRFloat) {        
    assert(LE->isIntegerType() && "LE must be an integer");
    return LeftFromRight(CK_IntegralToFloating);
  } 

  // just RE is floating
  if (IsLFloat) { 
    assert(RE->isIntegerType() && "RE must be an integer");
    return RightFromLeft(CK_IntegralToFloating);
  }
  
  // LE and RE are integer
  // The rules for this case are exactly as in C99 6.3.1.8
  int order = S.Context.getIntegerTypeOrder(LE, RE);
  bool LHSSigned = LE->hasSignedIntegerRepresentation();
  bool RHSSigned = RE->hasSignedIntegerRepresentation();
  if (LHSSigned == RHSSigned) {
    // Same signedness; use the higher-ranked type
    if (order >= 0) return RightFromLeft(CK_IntegralCast);

    return LeftFromRight(CK_IntegralCast);
  } 

  assert(LHSSigned ^ RHSSigned && "exactly one of two must be true");

  if (order != (LHSSigned ? 1 : -1)) {
    // The unsigned type has greater than or equal rank to the
    // signed type, so use the unsigned type
    if (RHSSigned) return RightFromLeft(CK_IntegralCast);

    return LeftFromRight(CK_IntegralCast);
  } 
  
  if (S.Context.getIntWidth(LE) != S.Context.getIntWidth(RE)) {
    // The two types are different widths; if we are here, that
    // means the signed type is larger than the unsigned type, so
    // use the signed type.
    if (LHSSigned) return RightFromLeft(CK_IntegralCast);

    return LeftFromRight(CK_IntegralCast);
  } 

  // The signed type is higher-ranked than the unsigned type,
  // but isn't actually any bigger (like unsigned int and long
  // on most 32-bit systems).  Use the unsigned type corresponding
  // to the signed type.
  LE = S.Context.getCorrespondingUnsignedType(LHSSigned ? LE : RE);
  RightFromLeft(CK_IntegralCast);
  return LeftFromRight(CK_IntegralCast);
}

/// Easy interface for checking sierra vector operands
QualType CheckSierraVectorOperands(Sema &S, ExprResult &LHS, ExprResult &RHS, 
                                   SourceLocation Loc, bool IsCompAssign) {
  SierraVectorOperandsChecker Checker(S, LHS, RHS, Loc, IsCompAssign);
  return Checker.Check();
}

//------------------------------------------------------------------------------

/// getSierraVectorType - Return the unique reference to an extended vector type of
/// the specified element type and size. VectorType must be a built-in type.
QualType ASTContext::getSierraVectorType(QualType vecType, unsigned NumElts) const {
  //assert(vecType->isBuiltinType() || vecType->isDependentType());

  // Check if we've already instantiated a vector of this type.
  llvm::FoldingSetNodeID ID;
  VectorType::Profile(ID, vecType, NumElts, Type::SierraVector,
                      VectorType::GenericVector);
  void *InsertPos = 0;
  if (VectorType *VTP = VectorTypes.FindNodeOrInsertPos(ID, InsertPos))
    return QualType(VTP, 0);

  // If the element type isn't canonical, this won't be a canonical type either,
  // so fill in the canonical type field.
  QualType Canonical;
  if (!vecType.isCanonical()) {
    Canonical = getSierraVectorType(getCanonicalType(vecType), NumElts);

    // Get the new insert position for the node we care about.
    VectorType *NewIP = VectorTypes.FindNodeOrInsertPos(ID, InsertPos);
    assert(NewIP == 0 && "Shouldn't be in the map!"); (void)NewIP;
  }
  SierraVectorType *New = new (*this, TypeAlignment)
    SierraVectorType(vecType, NumElts, Canonical);
  VectorTypes.InsertNode(New, InsertPos);
  Types.push_back(New);
  return QualType(New, 0);
}

//------------------------------------------------------------------------------

/// \brief Build an sierra vector type.
///
/// Run the required checks for the sierra vector type.
QualType BuildSierraVectorType(Sema &S, QualType T, Expr *ArraySize,
                                     SourceLocation AttrLoc) {
  // TODO allow more types
  if (!T->isDependentType() &&
      !T->isIntegerType() && !T->isRealFloatingType() && !T->isPointerType()) {
    S.Diag(AttrLoc, diag::err_attribute_invalid_vector_type) << T;
    return QualType();
  }

  if (!ArraySize->isTypeDependent() && !ArraySize->isValueDependent()) {
    llvm::APSInt vecSize(32);
    if (!ArraySize->isIntegerConstantExpr(vecSize, S.Context)) {
      S.Diag(AttrLoc, diag::err_attribute_argument_not_int)
        << "sierra_vector" << ArraySize->getSourceRange();
      return QualType();
    }

    // unlike gcc's vector_size attribute, the size is specified as the
    // number of elements, not the number of bytes.
    unsigned VecS = static_cast<unsigned>(vecSize.getZExtValue());

    if (VecS == 0) {
      S.Diag(AttrLoc, diag::err_attribute_zero_size)
      << ArraySize->getSourceRange();
      return QualType();
    }
    if (!llvm::isPowerOf2_32(VecS)){
      S.Diag(AttrLoc, diag::err_sierra_non_pow2) << VecS;
      return QualType();
    }

    // uniform special case
    if (VecS == 1)
      return QualType();

    unsigned CurS = S.getCurScope()->getCurrentVectorLength();
    // TODO polymorphism
    if (CurS != 1 && CurS != VecS) {
      S.Diag(AttrLoc, diag::err_sierra_incompatible_vector_lengths_in_decl)
        << CurS << VecS;
      return QualType();
    }

    QualType res = S.Context.getSierraVectorType(T, VecS);
    return res;
  }

  return S.Context.getDependentSizedSierraVectorType(T, ArraySize, AttrLoc);
}

//------------------------------------------------------------------------------

void HandleSierraVectorAttr(Sema &S, QualType& CurType, const AttributeList &Attr) {
  if (!S.getLangOpts().SIERRA) {
    S.Diag(Attr.getLoc(), diag::err_sierra_attr_not_enabled) << "sierra_vector";
    return;
  }

  Expr *sizeExpr;
  
  // Special case where the argument is a template id.
  if (Attr.getParameterName()) {
    CXXScopeSpec SS;
    SourceLocation TemplateKWLoc;
    UnqualifiedId id;
    id.setIdentifier(Attr.getParameterName(), Attr.getLoc());

    ExprResult Size = S.ActOnIdExpression(S.getCurScope(), SS, TemplateKWLoc,
                                          id, false, false);
    if (Size.isInvalid())
      return;
    
    sizeExpr = Size.get();
  } else {
    // check the attribute arguments.
    if (Attr.getNumArgs() != 1) {
      S.Diag(Attr.getLoc(), diag::err_attribute_wrong_number_arguments) << 1;
      return;
    }
    sizeExpr = Attr.getArg(0);
  }
  
  // Create the vector type.
  QualType T = BuildSierraVectorType(S, CurType, sizeExpr, Attr.getLoc());
  if (!T.isNull())
    CurType = T;
}

//------------------------------------------------------------------------------

bool HandleSierraSpmdAttr(Sema &S, const FunctionType *FunT, 
                          const AttributeList &Attr, unsigned &SpmdSize) {
  if (!S.getLangOpts().SIERRA) {
    S.Diag(Attr.getLoc(), diag::err_sierra_attr_not_enabled) << "sierra_spmd";
    return false;
  }

  if (!FunT)
    return false; // error message issued by usual clang magic

  bool result = true;

  if (Attr.getNumArgs() != 1) {
    S.Diag(Attr.getLoc(), diag::err_attribute_wrong_number_arguments) << 1;
    result = false;
  }

  Expr *SizeExpr;

  // Special case where the argument is a template id.
  if (Attr.getParameterName()) {
    CXXScopeSpec SS;
    SourceLocation TemplateKWLoc;
    UnqualifiedId id;
    id.setIdentifier(Attr.getParameterName(), Attr.getLoc());

    ExprResult Size = S.ActOnIdExpression(S.getCurScope(), SS, TemplateKWLoc,
                                          id, false, false);
    if (Size.isInvalid())
      return false;
    
    SizeExpr = Size.get();
  } else
    SizeExpr = Attr.getArg(0);

  if (SizeExpr->isTypeDependent() || SizeExpr->isValueDependent())
    result = false;

  if (!result)
    return false;

  llvm::APSInt SpmdSizeAPS(32);
  if (!SizeExpr->isIntegerConstantExpr(SpmdSizeAPS, S.Context)) {
    S.Diag(Attr.getLoc(), diag::err_attribute_argument_not_int)
      << "sierra_vector" << SizeExpr->getSourceRange();
    return false;
  }

  // check vector length of params/ret type
  // TODO -- how do I do that?

  // update current scope
  SpmdSize = static_cast<unsigned>(SpmdSizeAPS.getZExtValue());
  S.getCurScope()->setCurrentVectorLength(SpmdSize);
  return true;
}

//------------------------------------------------------------------------------

} // end namespace clang
