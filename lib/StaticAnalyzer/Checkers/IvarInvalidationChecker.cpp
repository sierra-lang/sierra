//=- IvarInvalidationChecker.cpp - -*- C++ -------------------------------*-==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//  This checker implements annotation driven invalidation checking. If a class
//  contains a method annotated with 'objc_instance_variable_invalidator',
//  - (void) foo
//           __attribute__((annotate("objc_instance_variable_invalidator")));
//  all the "ivalidatable" instance variables of this class should be
//  invalidated. We call an instance variable ivalidatable if it is an object of
//  a class which contains an invalidation method. There could be multiple
//  methods annotated with such annotations per class, either one can be used
//  to invalidate the ivar. An ivar or property are considered to be
//  invalidated if they are being assigned 'nil' or an invalidation method has
//  been called on them. An invalidation method should either invalidate all
//  the ivars or call another invalidation method (on self).
//
//===----------------------------------------------------------------------===//

#include "ClangSACheckers.h"
#include "clang/AST/Attr.h"
#include "clang/AST/DeclObjC.h"
#include "clang/AST/StmtVisitor.h"
#include "clang/StaticAnalyzer/Core/BugReporter/BugReporter.h"
#include "clang/StaticAnalyzer/Core/Checker.h"
#include "clang/StaticAnalyzer/Core/PathSensitive/AnalysisManager.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/ADT/SmallString.h"

using namespace clang;
using namespace ento;

namespace {
class IvarInvalidationChecker :
  public Checker<check::ASTDecl<ObjCImplementationDecl> > {

  typedef llvm::SmallSetVector<const ObjCMethodDecl*, 2> MethodSet;
  typedef llvm::DenseMap<const ObjCMethodDecl*,
                         const ObjCIvarDecl*> MethToIvarMapTy;
  typedef llvm::DenseMap<const ObjCPropertyDecl*,
                         const ObjCIvarDecl*> PropToIvarMapTy;
  typedef llvm::DenseMap<const ObjCIvarDecl*,
                         const ObjCPropertyDecl*> IvarToPropMapTy;


  struct InvalidationInfo {
    /// Has the ivar been invalidated?
    bool IsInvalidated;

    /// The methods which can be used to invalidate the ivar.
    MethodSet InvalidationMethods;

    InvalidationInfo() : IsInvalidated(false) {}
    void addInvalidationMethod(const ObjCMethodDecl *MD) {
      InvalidationMethods.insert(MD);
    }

    bool needsInvalidation() const {
      return !InvalidationMethods.empty();
    }

    void markInvalidated() {
      IsInvalidated = true;
    }

    bool markInvalidated(const ObjCMethodDecl *MD) {
      if (IsInvalidated)
        return true;
      for (MethodSet::iterator I = InvalidationMethods.begin(),
          E = InvalidationMethods.end(); I != E; ++I) {
        if (*I == MD) {
          IsInvalidated = true;
          return true;
        }
      }
      return false;
    }

    bool isInvalidated() const {
      return IsInvalidated;
    }
  };

  typedef llvm::DenseMap<const ObjCIvarDecl*, InvalidationInfo> IvarSet;

  /// Statement visitor, which walks the method body and flags the ivars
  /// referenced in it (either directly or via property).
  class MethodCrawler : public ConstStmtVisitor<MethodCrawler> {
    /// The set of Ivars which need to be invalidated.
    IvarSet &IVars;

    /// Flag is set as the result of a message send to another
    /// invalidation method.
    bool &CalledAnotherInvalidationMethod;

    /// Property setter to ivar mapping.
    const MethToIvarMapTy &PropertySetterToIvarMap;

    /// Property getter to ivar mapping.
    const MethToIvarMapTy &PropertyGetterToIvarMap;

    /// Property to ivar mapping.
    const PropToIvarMapTy &PropertyToIvarMap;

    /// The invalidation method being currently processed.
    const ObjCMethodDecl *InvalidationMethod;

    ASTContext &Ctx;

    /// Peel off parens, casts, OpaqueValueExpr, and PseudoObjectExpr.
    const Expr *peel(const Expr *E) const;

    /// Does this expression represent zero: '0'?
    bool isZero(const Expr *E) const;

    /// Mark the given ivar as invalidated.
    void markInvalidated(const ObjCIvarDecl *Iv);

    /// Checks if IvarRef refers to the tracked IVar, if yes, marks it as
    /// invalidated.
    void checkObjCIvarRefExpr(const ObjCIvarRefExpr *IvarRef);

    /// Checks if ObjCPropertyRefExpr refers to the tracked IVar, if yes, marks
    /// it as invalidated.
    void checkObjCPropertyRefExpr(const ObjCPropertyRefExpr *PA);

    /// Checks if ObjCMessageExpr refers to (is a getter for) the tracked IVar,
    /// if yes, marks it as invalidated.
    void checkObjCMessageExpr(const ObjCMessageExpr *ME);

    /// Checks if the Expr refers to an ivar, if yes, marks it as invalidated.
    void check(const Expr *E);

  public:
    MethodCrawler(IvarSet &InIVars,
                  bool &InCalledAnotherInvalidationMethod,
                  const MethToIvarMapTy &InPropertySetterToIvarMap,
                  const MethToIvarMapTy &InPropertyGetterToIvarMap,
                  const PropToIvarMapTy &InPropertyToIvarMap,
                  ASTContext &InCtx)
    : IVars(InIVars),
      CalledAnotherInvalidationMethod(InCalledAnotherInvalidationMethod),
      PropertySetterToIvarMap(InPropertySetterToIvarMap),
      PropertyGetterToIvarMap(InPropertyGetterToIvarMap),
      PropertyToIvarMap(InPropertyToIvarMap),
      InvalidationMethod(0),
      Ctx(InCtx) {}

    void VisitStmt(const Stmt *S) { VisitChildren(S); }

    void VisitBinaryOperator(const BinaryOperator *BO);

    void VisitObjCMessageExpr(const ObjCMessageExpr *ME);

    void VisitChildren(const Stmt *S) {
      for (Stmt::const_child_range I = S->children(); I; ++I) {
        if (*I)
          this->Visit(*I);
        if (CalledAnotherInvalidationMethod)
          return;
      }
    }
  };

  /// Check if the any of the methods inside the interface are annotated with
  /// the invalidation annotation, update the IvarInfo accordingly.
  static void containsInvalidationMethod(const ObjCContainerDecl *D,
                                         InvalidationInfo &Out);

  /// Check if ivar should be tracked and add to TrackedIvars if positive.
  /// Returns true if ivar should be tracked.
  static bool trackIvar(const ObjCIvarDecl *Iv, IvarSet &TrackedIvars,
                        const ObjCIvarDecl **FirstIvarDecl);

  /// Given the property declaration, and the list of tracked ivars, finds
  /// the ivar backing the property when possible. Returns '0' when no such
  /// ivar could be found.
  static const ObjCIvarDecl *findPropertyBackingIvar(
      const ObjCPropertyDecl *Prop,
      const ObjCInterfaceDecl *InterfaceD,
      IvarSet &TrackedIvars,
      const ObjCIvarDecl **FirstIvarDecl);

  /// Print ivar name or the property if the given ivar backs a property.
  static void printIvar(llvm::raw_svector_ostream &os,
                        const ObjCIvarDecl *IvarDecl,
                        IvarToPropMapTy &IvarToPopertyMap);
public:
  void checkASTDecl(const ObjCImplementationDecl *D, AnalysisManager& Mgr,
                    BugReporter &BR) const;
};

static bool isInvalidationMethod(const ObjCMethodDecl *M) {
  for (specific_attr_iterator<AnnotateAttr>
       AI = M->specific_attr_begin<AnnotateAttr>(),
       AE = M->specific_attr_end<AnnotateAttr>(); AI != AE; ++AI) {
    const AnnotateAttr *Ann = *AI;
    if (Ann->getAnnotation() == "objc_instance_variable_invalidator")
      return true;
  }
  return false;
}

void IvarInvalidationChecker::containsInvalidationMethod(
    const ObjCContainerDecl *D, InvalidationInfo &OutInfo) {

  if (!D)
    return;

  assert(!isa<ObjCImplementationDecl>(D));
  // TODO: Cache the results.

  // Check all methods.
  for (ObjCContainerDecl::method_iterator
      I = D->meth_begin(),
      E = D->meth_end(); I != E; ++I) {
      const ObjCMethodDecl *MDI = *I;
      if (isInvalidationMethod(MDI))
        OutInfo.addInvalidationMethod(
                               cast<ObjCMethodDecl>(MDI->getCanonicalDecl()));
  }

  // If interface, check all parent protocols and super.
  if (const ObjCInterfaceDecl *InterfD = dyn_cast<ObjCInterfaceDecl>(D)) {

    // Visit all protocols.
    for (ObjCInterfaceDecl::protocol_iterator
        I = InterfD->protocol_begin(),
        E = InterfD->protocol_end(); I != E; ++I) {
      containsInvalidationMethod((*I)->getDefinition(), OutInfo);
    }

    // Visit all categories in case the invalidation method is declared in
    // a category.
    for (ObjCInterfaceDecl::visible_extensions_iterator
           Ext = InterfD->visible_extensions_begin(),
           ExtEnd = InterfD->visible_extensions_end();
         Ext != ExtEnd; ++Ext) {
      containsInvalidationMethod(*Ext, OutInfo);
    }

    containsInvalidationMethod(InterfD->getSuperClass(), OutInfo);
    return;
  }

  // If protocol, check all parent protocols.
  if (const ObjCProtocolDecl *ProtD = dyn_cast<ObjCProtocolDecl>(D)) {
    for (ObjCInterfaceDecl::protocol_iterator
        I = ProtD->protocol_begin(),
        E = ProtD->protocol_end(); I != E; ++I) {
      containsInvalidationMethod((*I)->getDefinition(), OutInfo);
    }
    return;
  }

  return;
}

bool IvarInvalidationChecker::trackIvar(const ObjCIvarDecl *Iv,
                                        IvarSet &TrackedIvars,
                                        const ObjCIvarDecl **FirstIvarDecl) {
  QualType IvQTy = Iv->getType();
  const ObjCObjectPointerType *IvTy = IvQTy->getAs<ObjCObjectPointerType>();
  if (!IvTy)
    return false;
  const ObjCInterfaceDecl *IvInterf = IvTy->getInterfaceDecl();

  InvalidationInfo Info;
  containsInvalidationMethod(IvInterf, Info);
  if (Info.needsInvalidation()) {
    const ObjCIvarDecl *I = cast<ObjCIvarDecl>(Iv->getCanonicalDecl());
    TrackedIvars[I] = Info;
    if (!*FirstIvarDecl)
      *FirstIvarDecl = I;
    return true;
  }
  return false;
}

const ObjCIvarDecl *IvarInvalidationChecker::findPropertyBackingIvar(
                        const ObjCPropertyDecl *Prop,
                        const ObjCInterfaceDecl *InterfaceD,
                        IvarSet &TrackedIvars,
                        const ObjCIvarDecl **FirstIvarDecl) {
  const ObjCIvarDecl *IvarD = 0;

  // Lookup for the synthesized case.
  IvarD = Prop->getPropertyIvarDecl();
  // We only track the ivars/properties that are defined in the current 
  // class (not the parent).
  if (IvarD && IvarD->getContainingInterface() == InterfaceD) {
    if (TrackedIvars.count(IvarD)) {
      return IvarD;
    }
    // If the ivar is synthesized we still want to track it.
    if (trackIvar(IvarD, TrackedIvars, FirstIvarDecl))
      return IvarD;
  }

  // Lookup IVars named "_PropName"or "PropName" among the tracked Ivars.
  StringRef PropName = Prop->getIdentifier()->getName();
  for (IvarSet::const_iterator I = TrackedIvars.begin(),
                               E = TrackedIvars.end(); I != E; ++I) {
    const ObjCIvarDecl *Iv = I->first;
    StringRef IvarName = Iv->getName();

    if (IvarName == PropName)
      return Iv;

    SmallString<128> PropNameWithUnderscore;
    {
      llvm::raw_svector_ostream os(PropNameWithUnderscore);
      os << '_' << PropName;
    }
    if (IvarName == PropNameWithUnderscore.str())
      return Iv;
  }

  // Note, this is a possible source of false positives. We could look at the
  // getter implementation to find the ivar when its name is not derived from
  // the property name.
  return 0;
}

void IvarInvalidationChecker::printIvar(llvm::raw_svector_ostream &os,
                                        const ObjCIvarDecl *IvarDecl,
                                        IvarToPropMapTy &IvarToPopertyMap) {
  if (IvarDecl->getSynthesize()) {
    const ObjCPropertyDecl *PD = IvarToPopertyMap[IvarDecl];
    assert(PD &&"Do we synthesize ivars for something other than properties?");
    os << "Property "<< PD->getName() << " ";
  } else {
    os << "Instance variable "<< IvarDecl->getName() << " ";
  }
}

// Check that the invalidatable interfaces with ivars/properties implement the
// invalidation methods.
void IvarInvalidationChecker::checkASTDecl(const ObjCImplementationDecl *ImplD,
                                          AnalysisManager& Mgr,
                                          BugReporter &BR) const {
  // Collect all ivars that need cleanup.
  IvarSet Ivars;
  // Record the first Ivar needing invalidation; used in reporting when only
  // one ivar is sufficient. Cannot grab the first on the Ivars set to ensure
  // deterministic output.
  const ObjCIvarDecl *FirstIvarDecl = 0;
  const ObjCInterfaceDecl *InterfaceD = ImplD->getClassInterface();

  // Collect ivars declared in this class, its extensions and its implementation
  ObjCInterfaceDecl *IDecl = const_cast<ObjCInterfaceDecl *>(InterfaceD);
  for (const ObjCIvarDecl *Iv = IDecl->all_declared_ivar_begin(); Iv;
       Iv= Iv->getNextIvar())
    trackIvar(Iv, Ivars, &FirstIvarDecl);

  // Construct Property/Property Accessor to Ivar maps to assist checking if an
  // ivar which is backing a property has been reset.
  MethToIvarMapTy PropSetterToIvarMap;
  MethToIvarMapTy PropGetterToIvarMap;
  PropToIvarMapTy PropertyToIvarMap;
  IvarToPropMapTy IvarToPopertyMap;

  ObjCInterfaceDecl::PropertyMap PropMap;
  InterfaceD->collectPropertiesToImplement(PropMap);

  for (ObjCInterfaceDecl::PropertyMap::iterator
      I = PropMap.begin(), E = PropMap.end(); I != E; ++I) {
    const ObjCPropertyDecl *PD = I->second;

    const ObjCIvarDecl *ID = findPropertyBackingIvar(PD, InterfaceD, Ivars,
                                                     &FirstIvarDecl);
    if (!ID) {
      continue;
    }

    // Store the mappings.
    PD = cast<ObjCPropertyDecl>(PD->getCanonicalDecl());
    PropertyToIvarMap[PD] = ID;
    IvarToPopertyMap[ID] = PD;

    // Find the setter and the getter.
    const ObjCMethodDecl *SetterD = PD->getSetterMethodDecl();
    if (SetterD) {
      SetterD = cast<ObjCMethodDecl>(SetterD->getCanonicalDecl());
      PropSetterToIvarMap[SetterD] = ID;
    }

    const ObjCMethodDecl *GetterD = PD->getGetterMethodDecl();
    if (GetterD) {
      GetterD = cast<ObjCMethodDecl>(GetterD->getCanonicalDecl());
      PropGetterToIvarMap[GetterD] = ID;
    }
  }

  // If no ivars need invalidation, there is nothing to check here.
  if (Ivars.empty())
    return;

  // Find all invalidation methods in this @interface declaration and parents.
  InvalidationInfo Info;
  containsInvalidationMethod(InterfaceD, Info);

  // Report an error in case none of the invalidation methods are declared.
  if (!Info.needsInvalidation()) {
    SmallString<128> sbuf;
    llvm::raw_svector_ostream os(sbuf);
    assert(FirstIvarDecl);
    printIvar(os, FirstIvarDecl, IvarToPopertyMap);
    os << "needs to be invalidated; ";
    os << "no invalidation method is declared for " << InterfaceD->getName();

    PathDiagnosticLocation IvarDecLocation =
      PathDiagnosticLocation::createBegin(FirstIvarDecl, BR.getSourceManager());

    BR.EmitBasicReport(FirstIvarDecl, "Incomplete invalidation",
                       categories::CoreFoundationObjectiveC, os.str(),
                       IvarDecLocation);
    return;
  }

  // Check that all ivars are invalidated by the invalidation methods.
  bool AtImplementationContainsAtLeastOneInvalidationMethod = false;
  for (MethodSet::iterator I = Info.InvalidationMethods.begin(),
                           E = Info.InvalidationMethods.end(); I != E; ++I) {
    const ObjCMethodDecl *InterfD = *I;

    // Get the corresponding method in the @implementation.
    const ObjCMethodDecl *D = ImplD->getMethod(InterfD->getSelector(),
                                               InterfD->isInstanceMethod());
    if (D && D->hasBody()) {
      AtImplementationContainsAtLeastOneInvalidationMethod = true;

      // Get a copy of ivars needing invalidation.
      IvarSet IvarsI = Ivars;

      bool CalledAnotherInvalidationMethod = false;
      MethodCrawler(IvarsI,
                    CalledAnotherInvalidationMethod,
                    PropSetterToIvarMap,
                    PropGetterToIvarMap,
                    PropertyToIvarMap,
                    BR.getContext()).VisitStmt(D->getBody());
      // If another invalidation method was called, trust that full invalidation
      // has occurred.
      if (CalledAnotherInvalidationMethod)
        continue;

      // Warn on the ivars that were not invalidated by the method.
      for (IvarSet::const_iterator I = IvarsI.begin(),
                                   E = IvarsI.end(); I != E; ++I)
        if (!I->second.isInvalidated()) {
          SmallString<128> sbuf;
          llvm::raw_svector_ostream os(sbuf);
          printIvar(os, I->first, IvarToPopertyMap);
          os << "needs to be invalidated or set to nil";
          PathDiagnosticLocation MethodDecLocation =
              PathDiagnosticLocation::createEnd(D->getBody(),
                                                BR.getSourceManager(),
                                                Mgr.getAnalysisDeclContext(D));
          BR.EmitBasicReport(D, "Incomplete invalidation",
                             categories::CoreFoundationObjectiveC, os.str(),
                             MethodDecLocation);
        }
    }
  }

  // Report an error in case none of the invalidation methods are implemented.
  if (!AtImplementationContainsAtLeastOneInvalidationMethod) {
    SmallString<128> sbuf;
    llvm::raw_svector_ostream os(sbuf);
    assert(FirstIvarDecl);
    printIvar(os, FirstIvarDecl, IvarToPopertyMap);
    os << "needs to be invalidated; ";
    os << "no invalidation method is defined in the @implementation for "
       << InterfaceD->getName();

    PathDiagnosticLocation IvarDecLocation =
        PathDiagnosticLocation::createBegin(FirstIvarDecl,
                                            BR.getSourceManager());
    BR.EmitBasicReport(FirstIvarDecl, "Incomplete invalidation",
                       categories::CoreFoundationObjectiveC, os.str(),
                       IvarDecLocation);
  }
}

void IvarInvalidationChecker::MethodCrawler::markInvalidated(
    const ObjCIvarDecl *Iv) {
  IvarSet::iterator I = IVars.find(Iv);
  if (I != IVars.end()) {
    // If InvalidationMethod is present, we are processing the message send and
    // should ensure we are invalidating with the appropriate method,
    // otherwise, we are processing setting to 'nil'.
    if (InvalidationMethod)
      I->second.markInvalidated(InvalidationMethod);
    else
      I->second.markInvalidated();
  }
}

const Expr *IvarInvalidationChecker::MethodCrawler::peel(const Expr *E) const {
  E = E->IgnoreParenCasts();
  if (const PseudoObjectExpr *POE = dyn_cast<PseudoObjectExpr>(E))
    E = POE->getSyntacticForm()->IgnoreParenCasts();
  if (const OpaqueValueExpr *OVE = dyn_cast<OpaqueValueExpr>(E))
    E = OVE->getSourceExpr()->IgnoreParenCasts();
  return E;
}

void IvarInvalidationChecker::MethodCrawler::checkObjCIvarRefExpr(
    const ObjCIvarRefExpr *IvarRef) {
  if (const Decl *D = IvarRef->getDecl())
    markInvalidated(cast<ObjCIvarDecl>(D->getCanonicalDecl()));
}

void IvarInvalidationChecker::MethodCrawler::checkObjCMessageExpr(
    const ObjCMessageExpr *ME) {
  const ObjCMethodDecl *MD = ME->getMethodDecl();
  if (MD) {
    MD = cast<ObjCMethodDecl>(MD->getCanonicalDecl());
    MethToIvarMapTy::const_iterator IvI = PropertyGetterToIvarMap.find(MD);
    if (IvI != PropertyGetterToIvarMap.end())
      markInvalidated(IvI->second);
  }
}

void IvarInvalidationChecker::MethodCrawler::checkObjCPropertyRefExpr(
    const ObjCPropertyRefExpr *PA) {

  if (PA->isExplicitProperty()) {
    const ObjCPropertyDecl *PD = PA->getExplicitProperty();
    if (PD) {
      PD = cast<ObjCPropertyDecl>(PD->getCanonicalDecl());
      PropToIvarMapTy::const_iterator IvI = PropertyToIvarMap.find(PD);
      if (IvI != PropertyToIvarMap.end())
        markInvalidated(IvI->second);
      return;
    }
  }

  if (PA->isImplicitProperty()) {
    const ObjCMethodDecl *MD = PA->getImplicitPropertySetter();
    if (MD) {
      MD = cast<ObjCMethodDecl>(MD->getCanonicalDecl());
      MethToIvarMapTy::const_iterator IvI =PropertyGetterToIvarMap.find(MD);
      if (IvI != PropertyGetterToIvarMap.end())
        markInvalidated(IvI->second);
      return;
    }
  }
}

bool IvarInvalidationChecker::MethodCrawler::isZero(const Expr *E) const {
  E = peel(E);

  return (E->isNullPointerConstant(Ctx, Expr::NPC_ValueDependentIsNotNull)
           != Expr::NPCK_NotNull);
}

void IvarInvalidationChecker::MethodCrawler::check(const Expr *E) {
  E = peel(E);

  if (const ObjCIvarRefExpr *IvarRef = dyn_cast<ObjCIvarRefExpr>(E)) {
    checkObjCIvarRefExpr(IvarRef);
    return;
  }

  if (const ObjCPropertyRefExpr *PropRef = dyn_cast<ObjCPropertyRefExpr>(E)) {
    checkObjCPropertyRefExpr(PropRef);
    return;
  }

  if (const ObjCMessageExpr *MsgExpr = dyn_cast<ObjCMessageExpr>(E)) {
    checkObjCMessageExpr(MsgExpr);
    return;
  }
}

void IvarInvalidationChecker::MethodCrawler::VisitBinaryOperator(
    const BinaryOperator *BO) {
  VisitStmt(BO);

  // Do we assign/compare against zero? If yes, check the variable we are
  // assigning to.
  BinaryOperatorKind Opcode = BO->getOpcode();
  if (Opcode != BO_Assign &&
      Opcode != BO_EQ &&
      Opcode != BO_NE)
    return;

  if (isZero(BO->getRHS())) {
      check(BO->getLHS());
      return;
  }

  if (Opcode != BO_Assign && isZero(BO->getLHS())) {
    check(BO->getRHS());
    return;
  }
}

void IvarInvalidationChecker::MethodCrawler::VisitObjCMessageExpr(
  const ObjCMessageExpr *ME) {
  const ObjCMethodDecl *MD = ME->getMethodDecl();
  const Expr *Receiver = ME->getInstanceReceiver();

  // Stop if we are calling '[self invalidate]'.
  if (Receiver && isInvalidationMethod(MD))
    if (Receiver->isObjCSelfExpr()) {
      CalledAnotherInvalidationMethod = true;
      return;
    }

  // Check if we call a setter and set the property to 'nil'.
  if (MD && (ME->getNumArgs() == 1) && isZero(ME->getArg(0))) {
    MD = cast<ObjCMethodDecl>(MD->getCanonicalDecl());
    MethToIvarMapTy::const_iterator IvI = PropertySetterToIvarMap.find(MD);
    if (IvI != PropertySetterToIvarMap.end()) {
      markInvalidated(IvI->second);
      return;
    }
  }

  // Check if we call the 'invalidation' routine on the ivar.
  if (Receiver) {
    InvalidationMethod = MD;
    check(Receiver->IgnoreParenCasts());
    InvalidationMethod = 0;
  }

  VisitStmt(ME);
}
}

// Register the checker.
void ento::registerIvarInvalidationChecker(CheckerManager &mgr) {
  mgr.registerChecker<IvarInvalidationChecker>();
}
