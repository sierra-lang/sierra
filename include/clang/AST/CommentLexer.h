//===--- CommentLexer.h - Lexer for structured comments ---------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//  This file defines lexer for structured comments and supporting token class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_AST_COMMENT_LEXER_H
#define LLVM_CLANG_AST_COMMENT_LEXER_H

#include "clang/Basic/SourceManager.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/raw_ostream.h"

namespace clang {
namespace comments {

class Lexer;
class TextTokenRetokenizer;

namespace tok {
enum TokenKind {
  eof,
  newline,
  text,
  command,
  verbatim_block_begin,
  verbatim_block_line,
  verbatim_block_end,
  verbatim_line_name,
  verbatim_line_text,
  html_start_tag,     // <tag
  html_ident,         // attr
  html_equals,        // =
  html_quoted_string, // "blah\"blah" or 'blah\'blah'
  html_greater,       // >
  html_slash_greater, // />
  html_end_tag        // </tag
};
} // end namespace tok

class CommentOptions {
public:
  bool Markdown;
};

/// \brief Comment token.
class Token {
  friend class Lexer;
  friend class TextTokenRetokenizer;

  /// The location of the token.
  SourceLocation Loc;

  /// The actual kind of the token.
  tok::TokenKind Kind;

  /// Length of the token spelling in comment.  Can be 0 for synthenized
  /// tokens.
  unsigned Length;

  /// Contains text value associated with a token.
  const char *TextPtr1;
  unsigned TextLen1;

public:
  SourceLocation getLocation() const LLVM_READONLY { return Loc; }
  void setLocation(SourceLocation SL) { Loc = SL; }

  SourceLocation getEndLocation() const LLVM_READONLY {
    if (Length == 0 || Length == 1)
      return Loc;
    return Loc.getLocWithOffset(Length - 1);
  }

  tok::TokenKind getKind() const LLVM_READONLY { return Kind; }
  void setKind(tok::TokenKind K) { Kind = K; }

  bool is(tok::TokenKind K) const LLVM_READONLY { return Kind == K; }
  bool isNot(tok::TokenKind K) const LLVM_READONLY { return Kind != K; }

  unsigned getLength() const LLVM_READONLY { return Length; }
  void setLength(unsigned L) { Length = L; }

  StringRef getText() const LLVM_READONLY {
    assert(is(tok::text));
    return StringRef(TextPtr1, TextLen1);
  }

  void setText(StringRef Text) {
    assert(is(tok::text));
    TextPtr1 = Text.data();
    TextLen1 = Text.size();
  }

  StringRef getCommandName() const LLVM_READONLY {
    assert(is(tok::command));
    return StringRef(TextPtr1, TextLen1);
  }

  void setCommandName(StringRef Name) {
    assert(is(tok::command));
    TextPtr1 = Name.data();
    TextLen1 = Name.size();
  }

  StringRef getVerbatimBlockName() const LLVM_READONLY {
    assert(is(tok::verbatim_block_begin) || is(tok::verbatim_block_end));
    return StringRef(TextPtr1, TextLen1);
  }

  void setVerbatimBlockName(StringRef Name) {
    assert(is(tok::verbatim_block_begin) || is(tok::verbatim_block_end));
    TextPtr1 = Name.data();
    TextLen1 = Name.size();
  }

  StringRef getVerbatimBlockText() const LLVM_READONLY {
    assert(is(tok::verbatim_block_line));
    return StringRef(TextPtr1, TextLen1);
  }

  void setVerbatimBlockText(StringRef Text) {
    assert(is(tok::verbatim_block_line));
    TextPtr1 = Text.data();
    TextLen1 = Text.size();
  }

  /// Returns the name of verbatim line command.
  StringRef getVerbatimLineName() const LLVM_READONLY {
    assert(is(tok::verbatim_line_name));
    return StringRef(TextPtr1, TextLen1);
  }

  void setVerbatimLineName(StringRef Name) {
    assert(is(tok::verbatim_line_name));
    TextPtr1 = Name.data();
    TextLen1 = Name.size();
  }

  StringRef getVerbatimLineText() const LLVM_READONLY {
    assert(is(tok::verbatim_line_text));
    return StringRef(TextPtr1, TextLen1);
  }

  void setVerbatimLineText(StringRef Text) {
    assert(is(tok::verbatim_line_text));
    TextPtr1 = Text.data();
    TextLen1 = Text.size();
  }

  StringRef getHTMLTagStartName() const LLVM_READONLY {
    assert(is(tok::html_start_tag));
    return StringRef(TextPtr1, TextLen1);
  }

  void setHTMLTagStartName(StringRef Name) {
    assert(is(tok::html_start_tag));
    TextPtr1 = Name.data();
    TextLen1 = Name.size();
  }

  StringRef getHTMLIdent() const LLVM_READONLY {
    assert(is(tok::html_ident));
    return StringRef(TextPtr1, TextLen1);
  }

  void setHTMLIdent(StringRef Name) {
    assert(is(tok::html_ident));
    TextPtr1 = Name.data();
    TextLen1 = Name.size();
  }

  StringRef getHTMLQuotedString() const LLVM_READONLY {
    assert(is(tok::html_quoted_string));
    return StringRef(TextPtr1, TextLen1);
  }

  void setHTMLQuotedString(StringRef Str) {
    assert(is(tok::html_quoted_string));
    TextPtr1 = Str.data();
    TextLen1 = Str.size();
  }

  StringRef getHTMLTagEndName() const LLVM_READONLY {
    assert(is(tok::html_end_tag));
    return StringRef(TextPtr1, TextLen1);
  }

  void setHTMLTagEndName(StringRef Name) {
    assert(is(tok::html_end_tag));
    TextPtr1 = Name.data();
    TextLen1 = Name.size();
  }

  void dump(const Lexer &L, const SourceManager &SM) const;
};

/// \brief Comment lexer.
class Lexer {
private:
  Lexer(const Lexer&);          // DO NOT IMPLEMENT
  void operator=(const Lexer&); // DO NOT IMPLEMENT

  const char *const BufferStart;
  const char *const BufferEnd;
  SourceLocation FileLoc;
  CommentOptions CommOpts;

  const char *BufferPtr;

  /// One past end pointer for the current comment.  For BCPL comments points
  /// to newline or BufferEnd, for C comments points to star in '*/'.
  const char *CommentEnd;

  enum LexerCommentState {
    LCS_BeforeComment,
    LCS_InsideBCPLComment,
    LCS_InsideCComment,
    LCS_BetweenComments
  };

  /// Low-level lexer state, track if we are inside or outside of comment.
  LexerCommentState CommentState;

  enum LexerState {
    /// Lexing normal comment text
    LS_Normal,

    /// Finished lexing verbatim block beginning command, will lex first body
    /// line.
    LS_VerbatimBlockFirstLine,

    /// Lexing verbatim block body line-by-line, skipping line-starting
    /// decorations.
    LS_VerbatimBlockBody,

    /// Finished lexing verbatim line beginning command, will lex text (one
    /// line).
    LS_VerbatimLineText,

    /// Finished lexing \verbatim <TAG \endverbatim part, lexing tag attributes.
    LS_HTMLStartTag,

    /// Finished lexing \verbatim </TAG \endverbatim part, lexing '>'.
    LS_HTMLEndTag
  };

  /// Current lexing mode.
  LexerState State;

  /// A verbatim-like block command eats every character (except line starting
  /// decorations) until matching end command is seen or comment end is hit.
  struct VerbatimBlockCommand {
    StringRef BeginName;
    StringRef EndName;
  };

  typedef SmallVector<VerbatimBlockCommand, 4> VerbatimBlockCommandVector;

  /// Registered verbatim-like block commands.
  VerbatimBlockCommandVector VerbatimBlockCommands;

  /// If State is LS_VerbatimBlock, contains the name of verbatim end
  /// command, including command marker.
  SmallString<16> VerbatimBlockEndCommandName;

  bool isVerbatimBlockCommand(StringRef BeginName, StringRef &EndName) const;

  /// A verbatim-like line command eats everything until a newline is seen or
  /// comment end is hit.
  struct VerbatimLineCommand {
    StringRef Name;
  };

  typedef SmallVector<VerbatimLineCommand, 4> VerbatimLineCommandVector;

  /// Registered verbatim-like line commands.
  VerbatimLineCommandVector VerbatimLineCommands;

  bool isVerbatimLineCommand(StringRef Name) const;

  void formTokenWithChars(Token &Result, const char *TokEnd,
                          tok::TokenKind Kind) {
    const unsigned TokLen = TokEnd - BufferPtr;
    Result.setLocation(getSourceLocation(BufferPtr));
    Result.setKind(Kind);
    Result.setLength(TokLen);
#ifndef NDEBUG
    Result.TextPtr1 = "<UNSET>";
    Result.TextLen1 = 7;
#endif
    BufferPtr = TokEnd;
  }

  SourceLocation getSourceLocation(const char *Loc) const {
    assert(Loc >= BufferStart && Loc <= BufferEnd &&
           "Location out of range for this buffer!");

    const unsigned CharNo = Loc - BufferStart;
    return FileLoc.getLocWithOffset(CharNo);
  }

  /// Eat string matching regexp \code \s*\* \endcode.
  void skipLineStartingDecorations();

  /// Lex stuff inside comments.  CommentEnd should be set correctly.
  void lexCommentText(Token &T);

  void setupAndLexVerbatimBlock(Token &T,
                                const char *TextBegin,
                                char Marker, StringRef EndName);

  void lexVerbatimBlockFirstLine(Token &T);

  void lexVerbatimBlockBody(Token &T);

  void setupAndLexVerbatimLine(Token &T, const char *TextBegin);

  void lexVerbatimLineText(Token &T);

  void setupAndLexHTMLStartTag(Token &T);

  void lexHTMLStartTag(Token &T);

  void setupAndLexHTMLEndTag(Token &T);

  void lexHTMLEndTag(Token &T);

public:
  Lexer(SourceLocation FileLoc, const CommentOptions &CommOpts,
        const char *BufferStart, const char *BufferEnd);

  void lex(Token &T);

  StringRef getSpelling(const Token &Tok,
                        const SourceManager &SourceMgr,
                        bool *Invalid = NULL) const;

  /// \brief Register a new verbatim block command.
  void addVerbatimBlockCommand(StringRef BeginName, StringRef EndName);

  /// \brief Register a new verbatim line command.
  void addVerbatimLineCommand(StringRef Name);
};

} // end namespace comments
} // end namespace clang

#endif

