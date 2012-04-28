#include <clang/Sema/Sema.h>

using namespace clang;
using namespace sema;

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
    IsCompAssign(IsCompAssign), Swapped(false) {

  assert((!IsCompAssign || !Swapped) && "an compassign expr may not be swapped");

  LHSType = S.Context.getCanonicalType(LHS.get()->getType()).getUnqualifiedType();
  RHSType = S.Context.getCanonicalType(RHS.get()->getType()).getUnqualifiedType();

  // Normalize to have the RHS always be a sierra vector
  if (RHSType->isSierraVectorType() && !IsCompAssign) {
    Swapped = true;
    std::swap(RHS, LHS);
    std::swap(RHSType, LHSType);
  }

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

