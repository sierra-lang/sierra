//===--- Comment.h - Comment AST nodes --------------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//  This file defines comment AST nodes.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_AST_COMMENT_H
#define LLVM_CLANG_AST_COMMENT_H

#include "clang/Basic/SourceLocation.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringRef.h"

namespace clang {
namespace comments {

/// Any part of the comment.
/// Abstract class.
class Comment {
protected:
  /// Preferred location to show caret.
  SourceLocation Loc;

  /// Source range of this AST node.
  SourceRange Range;

  class CommentBitfields {
    friend class Comment;

    /// Type of this AST node.
    unsigned Kind : 8;
  };
  enum { NumCommentBits = 8 };

  class InlineContentCommentBitfields {
    friend class InlineContentComment;

    unsigned : NumCommentBits;

    /// True if there is a newline after this inline content node.
    /// (There is no separate AST node for a newline.)
    unsigned HasTrailingNewline : 1;
  };
  enum { NumInlineContentCommentBits = NumCommentBits + 1 };

  class TextCommentBitfields {
    friend class TextComment;

    unsigned : NumInlineContentCommentBits;

    /// True if \c IsWhitespace field contains a valid value.
    mutable unsigned IsWhitespaceValid : 1;

    /// True if this comment AST node contains only whitespace.
    mutable unsigned IsWhitespace : 1;
  };
  enum { NumTextCommentBits = NumInlineContentCommentBits + 2 };

  class InlineCommandCommentBitfields {
    friend class InlineCommandComment;

    unsigned : NumInlineContentCommentBits;

    unsigned RenderKind : 2;
  };
  enum { NumInlineCommandCommentBits = NumInlineContentCommentBits + 1 };

  class HTMLStartTagCommentBitfields {
    friend class HTMLStartTagComment;

    unsigned : NumInlineContentCommentBits;

    /// True if this tag is self-closing (e. g., <br />).  This is based on tag
    /// spelling in comment (plain <br> would not set this flag).
    unsigned IsSelfClosing : 1;
  };
  enum { NumHTMLStartTagCommentBits = NumInlineContentCommentBits + 1 };

  class ParagraphCommentBitfields {
    friend class ParagraphComment;

    unsigned : NumCommentBits;

    /// True if \c IsWhitespace field contains a valid value.
    mutable unsigned IsWhitespaceValid : 1;

    /// True if this comment AST node contains only whitespace.
    mutable unsigned IsWhitespace : 1;
  };
  enum { NumParagraphCommentBits = NumCommentBits + 2 };

  class ParamCommandCommentBitfields {
    friend class ParamCommandComment;

    unsigned : NumCommentBits;

    /// Parameter passing direction, see ParamCommandComment::PassDirection.
    unsigned Direction : 2;

    /// True if direction was specified explicitly in the comment.
    unsigned IsDirectionExplicit : 1;
  };
  enum { NumParamCommandCommentBits = 11 };

  union {
    CommentBitfields CommentBits;
    InlineContentCommentBitfields InlineContentCommentBits;
    TextCommentBitfields TextCommentBits;
    InlineCommandCommentBitfields InlineCommandCommentBits;
    HTMLStartTagCommentBitfields HTMLStartTagCommentBits;
    ParagraphCommentBitfields ParagraphCommentBits;
    ParamCommandCommentBitfields ParamCommandCommentBits;
  };

  void setSourceRange(SourceRange SR) {
    Range = SR;
  }

  void setLocation(SourceLocation L) {
    Loc = L;
  }

public:
  enum CommentKind {
    NoCommentKind = 0,
#define COMMENT(CLASS, PARENT) CLASS##Kind,
#define COMMENT_RANGE(BASE, FIRST, LAST) \
    First##BASE##Constant=FIRST##Kind, Last##BASE##Constant=LAST##Kind,
#define LAST_COMMENT_RANGE(BASE, FIRST, LAST) \
    First##BASE##Constant=FIRST##Kind, Last##BASE##Constant=LAST##Kind
#define ABSTRACT_COMMENT(COMMENT)
#include "clang/AST/CommentNodes.inc"
  };

  Comment(CommentKind K,
          SourceLocation LocBegin,
          SourceLocation LocEnd) :
      Loc(LocBegin), Range(SourceRange(LocBegin, LocEnd)) {
    CommentBits.Kind = K;
  }

  CommentKind getCommentKind() const {
    return static_cast<CommentKind>(CommentBits.Kind);
  }

  const char *getCommentKindName() const;

  LLVM_ATTRIBUTE_USED void dump() const;
  LLVM_ATTRIBUTE_USED void dump(SourceManager &SM) const;
  void dump(llvm::raw_ostream &OS, SourceManager *SM) const;

  static bool classof(const Comment *) { return true; }

  SourceRange getSourceRange() const LLVM_READONLY { return Range; }

  SourceLocation getLocStart() const LLVM_READONLY {
    return Range.getBegin();
  }

  SourceLocation getLocEnd() const LLVM_READONLY {
    return Range.getEnd();
  }

  SourceLocation getLocation() const LLVM_READONLY { return Loc; }

  typedef Comment * const *child_iterator;

  child_iterator child_begin() const;
  child_iterator child_end() const;

  // TODO: const child iterator

  unsigned child_count() const {
    return child_end() - child_begin();
  }
};

/// Inline content (contained within a block).
/// Abstract class.
class InlineContentComment : public Comment {
protected:
  InlineContentComment(CommentKind K,
                       SourceLocation LocBegin,
                       SourceLocation LocEnd) :
      Comment(K, LocBegin, LocEnd) {
    InlineContentCommentBits.HasTrailingNewline = 0;
  }

public:
  static bool classof(const Comment *C) {
    return C->getCommentKind() >= FirstInlineContentCommentConstant &&
           C->getCommentKind() <= LastInlineContentCommentConstant;
  }

  static bool classof(const InlineContentComment *) { return true; }

  void addTrailingNewline() {
    InlineContentCommentBits.HasTrailingNewline = 1;
  }

  bool hasTrailingNewline() const {
    return InlineContentCommentBits.HasTrailingNewline;
  }
};

/// Plain text.
class TextComment : public InlineContentComment {
  StringRef Text;

public:
  TextComment(SourceLocation LocBegin,
              SourceLocation LocEnd,
              StringRef Text) :
      InlineContentComment(TextCommentKind, LocBegin, LocEnd),
      Text(Text) {
    TextCommentBits.IsWhitespaceValid = false;
  }

  static bool classof(const Comment *C) {
    return C->getCommentKind() == TextCommentKind;
  }

  static bool classof(const TextComment *) { return true; }

  child_iterator child_begin() const { return NULL; }

  child_iterator child_end() const { return NULL; }

  StringRef getText() const LLVM_READONLY { return Text; }

  bool isWhitespace() const {
    if (TextCommentBits.IsWhitespaceValid)
      return TextCommentBits.IsWhitespace;

    TextCommentBits.IsWhitespace = isWhitespaceNoCache();
    TextCommentBits.IsWhitespaceValid = true;
    return TextCommentBits.IsWhitespace;
  }

private:
  bool isWhitespaceNoCache() const;
};

/// A command with word-like arguments that is considered inline content.
class InlineCommandComment : public InlineContentComment {
public:
  struct Argument {
    SourceRange Range;
    StringRef Text;

    Argument(SourceRange Range, StringRef Text) : Range(Range), Text(Text) { }
  };

  /// The most appropriate rendering mode for this command, chosen on command
  /// semantics in Doxygen.
  enum RenderKind {
    RenderNormal,
    RenderBold,
    RenderMonospaced,
    RenderEmphasized
  };

protected:
  /// Command name.
  StringRef Name;

  /// Command arguments.
  llvm::ArrayRef<Argument> Args;

public:
  InlineCommandComment(SourceLocation LocBegin,
                       SourceLocation LocEnd,
                       StringRef Name,
                       RenderKind RK,
                       llvm::ArrayRef<Argument> Args) :
      InlineContentComment(InlineCommandCommentKind, LocBegin, LocEnd),
      Name(Name), Args(Args) {
    InlineCommandCommentBits.RenderKind = RK;
  }

  static bool classof(const Comment *C) {
    return C->getCommentKind() == InlineCommandCommentKind;
  }

  static bool classof(const InlineCommandComment *) { return true; }

  child_iterator child_begin() const { return NULL; }

  child_iterator child_end() const { return NULL; }

  StringRef getCommandName() const {
    return Name;
  }

  SourceRange getCommandNameRange() const {
    return SourceRange(getLocStart().getLocWithOffset(-1),
                       getLocEnd());
  }

  RenderKind getRenderKind() const {
    return static_cast<RenderKind>(InlineCommandCommentBits.RenderKind);
  }

  unsigned getNumArgs() const {
    return Args.size();
  }

  StringRef getArgText(unsigned Idx) const {
    return Args[Idx].Text;
  }

  SourceRange getArgRange(unsigned Idx) const {
    return Args[Idx].Range;
  }
};

/// Abstract class for opening and closing HTML tags.  HTML tags are always
/// treated as inline content (regardless HTML semantics); opening and closing
/// tags are not matched.
class HTMLTagComment : public InlineContentComment {
protected:
  StringRef TagName;
  SourceRange TagNameRange;

  HTMLTagComment(CommentKind K,
                 SourceLocation LocBegin,
                 SourceLocation LocEnd,
                 StringRef TagName,
                 SourceLocation TagNameBegin,
                 SourceLocation TagNameEnd) :
      InlineContentComment(K, LocBegin, LocEnd),
      TagName(TagName),
      TagNameRange(TagNameBegin, TagNameEnd) {
    setLocation(TagNameBegin);
  }

public:
  static bool classof(const Comment *C) {
    return C->getCommentKind() >= FirstHTMLTagCommentConstant &&
           C->getCommentKind() <= LastHTMLTagCommentConstant;
  }

  static bool classof(const HTMLTagComment *) { return true; }

  StringRef getTagName() const LLVM_READONLY { return TagName; }

  SourceRange getTagNameSourceRange() const LLVM_READONLY {
    SourceLocation L = getLocation();
    return SourceRange(L.getLocWithOffset(1),
                       L.getLocWithOffset(1 + TagName.size()));
  }
};

/// An opening HTML tag with attributes.
class HTMLStartTagComment : public HTMLTagComment {
public:
  class Attribute {
  public:
    SourceLocation NameLocBegin;
    StringRef Name;

    SourceLocation EqualsLoc;

    SourceRange ValueRange;
    StringRef Value;

    Attribute() { }

    Attribute(SourceLocation NameLocBegin, StringRef Name) :
        NameLocBegin(NameLocBegin), Name(Name),
        EqualsLoc(SourceLocation()),
        ValueRange(SourceRange()), Value(StringRef())
    { }

    Attribute(SourceLocation NameLocBegin, StringRef Name,
              SourceLocation EqualsLoc,
              SourceRange ValueRange, StringRef Value) :
        NameLocBegin(NameLocBegin), Name(Name),
        EqualsLoc(EqualsLoc),
        ValueRange(ValueRange), Value(Value)
    { }

    SourceLocation getNameLocEnd() const {
      return NameLocBegin.getLocWithOffset(Name.size());
    }

    SourceRange getNameRange() const {
      return SourceRange(NameLocBegin, getNameLocEnd());
    }
  };

private:
  ArrayRef<Attribute> Attributes;

public:
  HTMLStartTagComment(SourceLocation LocBegin,
                      StringRef TagName) :
      HTMLTagComment(HTMLStartTagCommentKind,
                     LocBegin, LocBegin.getLocWithOffset(1 + TagName.size()),
                     TagName,
                     LocBegin.getLocWithOffset(1),
                     LocBegin.getLocWithOffset(1 + TagName.size())) {
    HTMLStartTagCommentBits.IsSelfClosing = false;
  }

  static bool classof(const Comment *C) {
    return C->getCommentKind() == HTMLStartTagCommentKind;
  }

  static bool classof(const HTMLStartTagComment *) { return true; }

  child_iterator child_begin() const { return NULL; }

  child_iterator child_end() const { return NULL; }

  unsigned getNumAttrs() const {
    return Attributes.size();
  }

  const Attribute &getAttr(unsigned Idx) const {
    return Attributes[Idx];
  }

  void setAttrs(ArrayRef<Attribute> Attrs) {
    Attributes = Attrs;
    if (!Attrs.empty()) {
      const Attribute &Attr = Attrs.back();
      SourceLocation L = Attr.ValueRange.getEnd();
      if (L.isValid())
        Range.setEnd(L);
      else {
        Range.setEnd(Attr.getNameLocEnd());
      }
    }
  }

  void setGreaterLoc(SourceLocation GreaterLoc) {
    Range.setEnd(GreaterLoc);
  }

  bool isSelfClosing() const {
    return HTMLStartTagCommentBits.IsSelfClosing;
  }

  void setSelfClosing() {
    HTMLStartTagCommentBits.IsSelfClosing = true;
  }
};

/// A closing HTML tag.
class HTMLEndTagComment : public HTMLTagComment {
public:
  HTMLEndTagComment(SourceLocation LocBegin,
                    SourceLocation LocEnd,
                    StringRef TagName) :
      HTMLTagComment(HTMLEndTagCommentKind,
                     LocBegin, LocEnd,
                     TagName,
                     LocBegin.getLocWithOffset(2),
                     LocBegin.getLocWithOffset(2 + TagName.size()))
  { }

  static bool classof(const Comment *C) {
    return C->getCommentKind() == HTMLEndTagCommentKind;
  }

  static bool classof(const HTMLEndTagComment *) { return true; }

  child_iterator child_begin() const { return NULL; }

  child_iterator child_end() const { return NULL; }
};

/// Block content (contains inline content).
/// Abstract class.
class BlockContentComment : public Comment {
protected:
  BlockContentComment(CommentKind K,
                      SourceLocation LocBegin,
                      SourceLocation LocEnd) :
      Comment(K, LocBegin, LocEnd)
  { }

public:
  static bool classof(const Comment *C) {
    return C->getCommentKind() >= FirstBlockContentCommentConstant &&
           C->getCommentKind() <= LastBlockContentCommentConstant;
  }

  static bool classof(const BlockContentComment *) { return true; }
};

/// A single paragraph that contains inline content.
class ParagraphComment : public BlockContentComment {
  llvm::ArrayRef<InlineContentComment *> Content;

public:
  ParagraphComment(llvm::ArrayRef<InlineContentComment *> Content) :
      BlockContentComment(ParagraphCommentKind,
                          SourceLocation(),
                          SourceLocation()),
      Content(Content) {
    if (Content.empty()) {
      ParagraphCommentBits.IsWhitespace = true;
      ParagraphCommentBits.IsWhitespaceValid = true;
      return;
    }

    ParagraphCommentBits.IsWhitespaceValid = false;

    setSourceRange(SourceRange(Content.front()->getLocStart(),
                               Content.back()->getLocEnd()));
    setLocation(Content.front()->getLocStart());
  }

  static bool classof(const Comment *C) {
    return C->getCommentKind() == ParagraphCommentKind;
  }

  static bool classof(const ParagraphComment *) { return true; }

  child_iterator child_begin() const {
    return reinterpret_cast<child_iterator>(Content.begin());
  }

  child_iterator child_end() const {
    return reinterpret_cast<child_iterator>(Content.end());
  }

  bool isWhitespace() const {
    if (ParagraphCommentBits.IsWhitespaceValid)
      return ParagraphCommentBits.IsWhitespace;

    ParagraphCommentBits.IsWhitespace = isWhitespaceNoCache();
    ParagraphCommentBits.IsWhitespaceValid = true;
    return ParagraphCommentBits.IsWhitespace;
  }

private:
  bool isWhitespaceNoCache() const;
};

/// A command that has zero or more word-like arguments (number of word-like
/// arguments depends on command name) and a paragraph as an argument
/// (e. g., \\brief).
class BlockCommandComment : public BlockContentComment {
public:
  struct Argument {
    SourceRange Range;
    StringRef Text;

    Argument() { }
    Argument(SourceRange Range, StringRef Text) : Range(Range), Text(Text) { }
  };

protected:
  /// Command name.
  StringRef Name;

  /// Word-like arguments.
  llvm::ArrayRef<Argument> Args;

  /// Paragraph argument.
  ParagraphComment *Paragraph;

  BlockCommandComment(CommentKind K,
                      SourceLocation LocBegin,
                      SourceLocation LocEnd,
                      StringRef Name) :
      BlockContentComment(K, LocBegin, LocEnd),
      Name(Name),
      Paragraph(NULL) {
    setLocation(getCommandNameRange().getBegin());
  }

public:
  BlockCommandComment(SourceLocation LocBegin,
                      SourceLocation LocEnd,
                      StringRef Name) :
      BlockContentComment(BlockCommandCommentKind, LocBegin, LocEnd),
      Name(Name),
      Paragraph(NULL) {
    setLocation(getCommandNameRange().getBegin());
  }

  static bool classof(const Comment *C) {
    return C->getCommentKind() >= FirstBlockCommandCommentConstant &&
           C->getCommentKind() <= LastBlockCommandCommentConstant;
  }

  static bool classof(const BlockCommandComment *) { return true; }

  child_iterator child_begin() const {
    return reinterpret_cast<child_iterator>(&Paragraph);
  }

  child_iterator child_end() const {
    return reinterpret_cast<child_iterator>(&Paragraph + 1);
  }

  StringRef getCommandName() const {
    return Name;
  }

  SourceRange getCommandNameRange() const {
    return SourceRange(getLocStart().getLocWithOffset(1),
                       getLocStart().getLocWithOffset(1 + Name.size()));
  }

  unsigned getNumArgs() const {
    return Args.size();
  }

  StringRef getArgText(unsigned Idx) const {
    return Args[Idx].Text;
  }

  SourceRange getArgRange(unsigned Idx) const {
    return Args[Idx].Range;
  }

  void setArgs(llvm::ArrayRef<Argument> A) {
    Args = A;
    if (Args.size() > 0) {
      SourceLocation NewLocEnd = Args.back().Range.getEnd();
      if (NewLocEnd.isValid())
        setSourceRange(SourceRange(getLocStart(), NewLocEnd));
    }
  }

  ParagraphComment *getParagraph() const LLVM_READONLY {
    return Paragraph;
  }

  bool hasNonWhitespaceParagraph() const {
    return Paragraph && !Paragraph->isWhitespace();
  }

  void setParagraph(ParagraphComment *PC) {
    Paragraph = PC;
    SourceLocation NewLocEnd = PC->getLocEnd();
    if (NewLocEnd.isValid())
      setSourceRange(SourceRange(getLocStart(), NewLocEnd));
  }
};

/// Doxygen \\param command.
class ParamCommandComment : public BlockCommandComment {
private:
  /// Parameter index in the function declaration.
  unsigned ParamIndex;

public:
  enum { InvalidParamIndex = ~0U };

  ParamCommandComment(SourceLocation LocBegin,
                      SourceLocation LocEnd,
                      StringRef Name) :
      BlockCommandComment(ParamCommandCommentKind, LocBegin, LocEnd, Name),
      ParamIndex(InvalidParamIndex) {
    ParamCommandCommentBits.Direction = In;
    ParamCommandCommentBits.IsDirectionExplicit = false;
  }

  static bool classof(const Comment *C) {
    return C->getCommentKind() == ParamCommandCommentKind;
  }

  static bool classof(const ParamCommandComment *) { return true; }

  enum PassDirection {
    In,
    Out,
    InOut
  };

  static const char *getDirectionAsString(PassDirection D);

  PassDirection getDirection() const LLVM_READONLY {
    return static_cast<PassDirection>(ParamCommandCommentBits.Direction);
  }

  bool isDirectionExplicit() const LLVM_READONLY {
    return ParamCommandCommentBits.IsDirectionExplicit;
  }

  void setDirection(PassDirection Direction, bool Explicit) {
    ParamCommandCommentBits.Direction = Direction;
    ParamCommandCommentBits.IsDirectionExplicit = Explicit;
  }

  bool hasParamName() const {
    return getNumArgs() > 0;
  }

  StringRef getParamName() const {
    return Args[0].Text;
  }

  SourceRange getParamNameRange() const {
    return Args[0].Range;
  }

  bool isParamIndexValid() const LLVM_READONLY {
    return ParamIndex != InvalidParamIndex;
  }

  unsigned getParamIndex() const LLVM_READONLY {
    return ParamIndex;
  }

  void setParamIndex(unsigned Index) {
    ParamIndex = Index;
    assert(isParamIndexValid());
  }
};

/// A line of text contained in a verbatim block.
class VerbatimBlockLineComment : public Comment {
  StringRef Text;

public:
  VerbatimBlockLineComment(SourceLocation LocBegin,
                           StringRef Text) :
      Comment(VerbatimBlockLineCommentKind,
              LocBegin,
              LocBegin.getLocWithOffset(Text.size())),
      Text(Text)
  { }

  static bool classof(const Comment *C) {
    return C->getCommentKind() == VerbatimBlockLineCommentKind;
  }

  static bool classof(const VerbatimBlockLineComment *) { return true; }

  child_iterator child_begin() const { return NULL; }

  child_iterator child_end() const { return NULL; }

  StringRef getText() const LLVM_READONLY {
    return Text;
  }
};

/// A verbatim block command (e. g., preformatted code).  Verbatim block has an
/// opening and a closing command and contains multiple lines of text
/// (VerbatimBlockLineComment nodes).
class VerbatimBlockComment : public BlockCommandComment {
protected:
  StringRef CloseName;
  SourceLocation CloseNameLocBegin;
  llvm::ArrayRef<VerbatimBlockLineComment *> Lines;

public:
  VerbatimBlockComment(SourceLocation LocBegin,
                       SourceLocation LocEnd,
                       StringRef Name) :
      BlockCommandComment(VerbatimBlockCommentKind,
                          LocBegin, LocEnd, Name)
  { }

  static bool classof(const Comment *C) {
    return C->getCommentKind() == VerbatimBlockCommentKind;
  }

  static bool classof(const VerbatimBlockComment *) { return true; }

  child_iterator child_begin() const {
    return reinterpret_cast<child_iterator>(Lines.begin());
  }

  child_iterator child_end() const {
    return reinterpret_cast<child_iterator>(Lines.end());
  }

  void setCloseName(StringRef Name, SourceLocation LocBegin) {
    CloseName = Name;
    CloseNameLocBegin = LocBegin;
  }

  void setLines(llvm::ArrayRef<VerbatimBlockLineComment *> L) {
    Lines = L;
  }

  StringRef getCloseName() const {
    return CloseName;
  }

  unsigned getNumLines() const {
    return Lines.size();
  }

  StringRef getText(unsigned LineIdx) const {
    return Lines[LineIdx]->getText();
  }
};

/// A verbatim line command.  Verbatim line has an opening command, a single
/// line of text (up to the newline after the opening command) and has no
/// closing command.
class VerbatimLineComment : public BlockCommandComment {
protected:
  StringRef Text;
  SourceLocation TextBegin;

public:
  VerbatimLineComment(SourceLocation LocBegin,
                      SourceLocation LocEnd,
                      StringRef Name,
                      SourceLocation TextBegin,
                      StringRef Text) :
      BlockCommandComment(VerbatimLineCommentKind,
                          LocBegin, LocEnd,
                          Name),
      Text(Text),
      TextBegin(TextBegin)
  { }

  static bool classof(const Comment *C) {
    return C->getCommentKind() == VerbatimLineCommentKind;
  }

  static bool classof(const VerbatimLineComment *) { return true; }

  child_iterator child_begin() const { return NULL; }

  child_iterator child_end() const { return NULL; }

  StringRef getText() const {
    return Text;
  }

  SourceRange getTextRange() const {
    return SourceRange(TextBegin, getLocEnd());
  }
};

/// A full comment attached to a declaration, contains block content.
class FullComment : public Comment {
  llvm::ArrayRef<BlockContentComment *> Blocks;

public:
  FullComment(llvm::ArrayRef<BlockContentComment *> Blocks) :
      Comment(FullCommentKind, SourceLocation(), SourceLocation()),
      Blocks(Blocks) {
    if (Blocks.empty())
      return;

    setSourceRange(SourceRange(Blocks.front()->getLocStart(),
                               Blocks.back()->getLocEnd()));
    setLocation(Blocks.front()->getLocStart());
  }

  static bool classof(const Comment *C) {
    return C->getCommentKind() == FullCommentKind;
  }

  static bool classof(const FullComment *) { return true; }

  child_iterator child_begin() const {
    return reinterpret_cast<child_iterator>(Blocks.begin());
  }

  child_iterator child_end() const {
    return reinterpret_cast<child_iterator>(Blocks.end());
  }
};

} // end namespace comments
} // end namespace clang

#endif

