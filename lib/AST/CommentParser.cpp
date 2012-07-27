//===--- CommentParser.cpp - Doxygen comment parser -----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "clang/AST/CommentParser.h"
#include "clang/AST/CommentSema.h"
#include "clang/AST/CommentDiagnostic.h"
#include "clang/Basic/SourceManager.h"
#include "llvm/Support/ErrorHandling.h"

namespace clang {
namespace comments {

/// Re-lexes a sequence of tok::text tokens.
class TextTokenRetokenizer {
  llvm::BumpPtrAllocator &Allocator;
  Parser &P;

  /// This flag is set when there are no more tokens we can fetch from lexer.
  bool NoMoreInterestingTokens;

  /// Token buffer: tokens we have processed and lookahead.
  SmallVector<Token, 16> Toks;

  /// A position in \c Toks.
  struct Position {
    unsigned CurToken;
    const char *BufferStart;
    const char *BufferEnd;
    const char *BufferPtr;
    SourceLocation BufferStartLoc;
  };

  /// Current position in Toks.
  Position Pos;

  bool isEnd() const {
    return Pos.CurToken >= Toks.size();
  }

  /// Sets up the buffer pointers to point to current token.
  void setupBuffer() {
    assert(!isEnd());
    const Token &Tok = Toks[Pos.CurToken];

    Pos.BufferStart = Tok.getText().begin();
    Pos.BufferEnd = Tok.getText().end();
    Pos.BufferPtr = Pos.BufferStart;
    Pos.BufferStartLoc = Tok.getLocation();
  }

  SourceLocation getSourceLocation() const {
    const unsigned CharNo = Pos.BufferPtr - Pos.BufferStart;
    return Pos.BufferStartLoc.getLocWithOffset(CharNo);
  }

  char peek() const {
    assert(!isEnd());
    assert(Pos.BufferPtr != Pos.BufferEnd);
    return *Pos.BufferPtr;
  }

  void consumeChar() {
    assert(!isEnd());
    assert(Pos.BufferPtr != Pos.BufferEnd);
    Pos.BufferPtr++;
    if (Pos.BufferPtr == Pos.BufferEnd) {
      Pos.CurToken++;
      if (isEnd() && !addToken())
        return;

      assert(!isEnd());
      setupBuffer();
    }
  }

  /// Add a token.
  /// Returns true on success, false if there are no interesting tokens to
  /// fetch from lexer.
  bool addToken() {
    if (NoMoreInterestingTokens)
      return false;

    if (P.Tok.is(tok::newline)) {
      // If we see a single newline token between text tokens, skip it.
      Token Newline = P.Tok;
      P.consumeToken();
      if (P.Tok.isNot(tok::text)) {
        P.putBack(Newline);
        NoMoreInterestingTokens = true;
        return false;
      }
    }
    if (P.Tok.isNot(tok::text)) {
      NoMoreInterestingTokens = true;
      return false;
    }

    Toks.push_back(P.Tok);
    P.consumeToken();
    if (Toks.size() == 1)
      setupBuffer();
    return true;
  }

  static bool isWhitespace(char C) {
    return C == ' ' || C == '\n' || C == '\r' ||
           C == '\t' || C == '\f' || C == '\v';
  }

  void consumeWhitespace() {
    while (!isEnd()) {
      if (isWhitespace(peek()))
        consumeChar();
      else
        break;
    }
  }

  void formTokenWithChars(Token &Result,
                          SourceLocation Loc,
                          const char *TokBegin,
                          unsigned TokLength,
                          StringRef Text) {
    Result.setLocation(Loc);
    Result.setKind(tok::text);
    Result.setLength(TokLength);
#ifndef NDEBUG
    Result.TextPtr1 = "<UNSET>";
    Result.TextLen1 = 7;
#endif
    Result.setText(Text);
  }

public:
  TextTokenRetokenizer(llvm::BumpPtrAllocator &Allocator, Parser &P):
      Allocator(Allocator), P(P), NoMoreInterestingTokens(false) {
    Pos.CurToken = 0;
    addToken();
  }

  /// Extract a word -- sequence of non-whitespace characters.
  bool lexWord(Token &Tok) {
    if (isEnd())
      return false;

    Position SavedPos = Pos;

    consumeWhitespace();
    SmallString<32> WordText;
    const char *WordBegin = Pos.BufferPtr;
    SourceLocation Loc = getSourceLocation();
    while (!isEnd()) {
      const char C = peek();
      if (!isWhitespace(C)) {
        WordText.push_back(C);
        consumeChar();
      } else
        break;
    }
    const unsigned Length = WordText.size();
    if (Length == 0) {
      Pos = SavedPos;
      return false;
    }

    char *TextPtr = Allocator.Allocate<char>(Length + 1);

    memcpy(TextPtr, WordText.c_str(), Length + 1);
    StringRef Text = StringRef(TextPtr, Length);

    formTokenWithChars(Tok, Loc, WordBegin,
                       Pos.BufferPtr - WordBegin, Text);
    return true;
  }

  bool lexDelimitedSeq(Token &Tok, char OpenDelim, char CloseDelim) {
    if (isEnd())
      return false;

    Position SavedPos = Pos;

    consumeWhitespace();
    SmallString<32> WordText;
    const char *WordBegin = Pos.BufferPtr;
    SourceLocation Loc = getSourceLocation();
    bool Error = false;
    if (!isEnd()) {
      const char C = peek();
      if (C == OpenDelim) {
        WordText.push_back(C);
        consumeChar();
      } else
        Error = true;
    }
    char C = '\0';
    while (!Error && !isEnd()) {
      C = peek();
      WordText.push_back(C);
      consumeChar();
      if (C == CloseDelim)
        break;
    }
    if (!Error && C != CloseDelim)
      Error = true;

    if (Error) {
      Pos = SavedPos;
      return false;
    }

    const unsigned Length = WordText.size();
    char *TextPtr = Allocator.Allocate<char>(Length + 1);

    memcpy(TextPtr, WordText.c_str(), Length + 1);
    StringRef Text = StringRef(TextPtr, Length);

    formTokenWithChars(Tok, Loc, WordBegin,
                       Pos.BufferPtr - WordBegin, Text);
    return true;
  }

  /// Put back tokens that we didn't consume.
  void putBackLeftoverTokens() {
    if (isEnd())
      return;

    bool HavePartialTok = false;
    Token PartialTok;
    if (Pos.BufferPtr != Pos.BufferStart) {
      formTokenWithChars(PartialTok, getSourceLocation(),
                         Pos.BufferPtr, Pos.BufferEnd - Pos.BufferPtr,
                         StringRef(Pos.BufferPtr,
                                   Pos.BufferEnd - Pos.BufferPtr));
      HavePartialTok = true;
      Pos.CurToken++;
    }

    P.putBack(llvm::makeArrayRef(Toks.begin() + Pos.CurToken, Toks.end()));
    Pos.CurToken = Toks.size();

    if (HavePartialTok)
      P.putBack(PartialTok);
  }
};

Parser::Parser(Lexer &L, Sema &S, llvm::BumpPtrAllocator &Allocator,
               const SourceManager &SourceMgr, DiagnosticsEngine &Diags):
    L(L), S(S), Allocator(Allocator), SourceMgr(SourceMgr), Diags(Diags) {
  consumeToken();
}

ParamCommandComment *Parser::parseParamCommandArgs(
    ParamCommandComment *PC,
    TextTokenRetokenizer &Retokenizer) {
  Token Arg;
  // Check if argument looks like direction specification: [dir]
  // e.g., [in], [out], [in,out]
  if (Retokenizer.lexDelimitedSeq(Arg, '[', ']'))
    PC = S.actOnParamCommandDirectionArg(PC,
                                         Arg.getLocation(),
                                         Arg.getEndLocation(),
                                         Arg.getText());

  if (Retokenizer.lexWord(Arg))
    PC = S.actOnParamCommandParamNameArg(PC,
                                         Arg.getLocation(),
                                         Arg.getEndLocation(),
                                         Arg.getText());

  return PC;
}

BlockCommandComment *Parser::parseBlockCommandArgs(
    BlockCommandComment *BC,
    TextTokenRetokenizer &Retokenizer,
    unsigned NumArgs) {
  typedef BlockCommandComment::Argument Argument;
  Argument *Args =
      new (Allocator.Allocate<Argument>(NumArgs)) Argument[NumArgs];
  unsigned ParsedArgs = 0;
  Token Arg;
  while (ParsedArgs < NumArgs && Retokenizer.lexWord(Arg)) {
    Args[ParsedArgs] = Argument(SourceRange(Arg.getLocation(),
                                            Arg.getEndLocation()),
                                Arg.getText());
    ParsedArgs++;
  }

  return S.actOnBlockCommandArgs(BC, llvm::makeArrayRef(Args, ParsedArgs));
}

BlockCommandComment *Parser::parseBlockCommand() {
  assert(Tok.is(tok::command));

  ParamCommandComment *PC;
  BlockCommandComment *BC;
  bool IsParam = false;
  unsigned NumArgs = 0;
  if (S.isParamCommand(Tok.getCommandName())) {
    IsParam = true;
    PC = S.actOnParamCommandStart(Tok.getLocation(),
                                  Tok.getEndLocation(),
                                  Tok.getCommandName());
  } else {
    NumArgs = S.getBlockCommandNumArgs(Tok.getCommandName());
    BC = S.actOnBlockCommandStart(Tok.getLocation(),
                                  Tok.getEndLocation(),
                                  Tok.getCommandName());
  }
  consumeToken();

  if (Tok.is(tok::command) && S.isBlockCommand(Tok.getCommandName())) {
    // Block command ahead.  We can't nest block commands, so pretend that this
    // command has an empty argument.
    ParagraphComment *PC = S.actOnParagraphComment(
                                ArrayRef<InlineContentComment *>());
    return S.actOnBlockCommandFinish(BC, PC);
  }

  if (IsParam || NumArgs > 0) {
    // In order to parse command arguments we need to retokenize a few
    // following text tokens.
    TextTokenRetokenizer Retokenizer(Allocator, *this);

    if (IsParam)
      PC = parseParamCommandArgs(PC, Retokenizer);
    else
      BC = parseBlockCommandArgs(BC, Retokenizer, NumArgs);

    Retokenizer.putBackLeftoverTokens();
  }

  BlockContentComment *Block = parseParagraphOrBlockCommand();
  // Since we have checked for a block command, we should have parsed a
  // paragraph.
  if (IsParam)
    return S.actOnParamCommandFinish(PC, cast<ParagraphComment>(Block));
  else
    return S.actOnBlockCommandFinish(BC, cast<ParagraphComment>(Block));
}

InlineCommandComment *Parser::parseInlineCommand() {
  assert(Tok.is(tok::command));

  const Token CommandTok = Tok;
  consumeToken();

  TextTokenRetokenizer Retokenizer(Allocator, *this);

  Token ArgTok;
  bool ArgTokValid = Retokenizer.lexWord(ArgTok);

  InlineCommandComment *IC;
  if (ArgTokValid) {
    IC = S.actOnInlineCommand(CommandTok.getLocation(),
                              CommandTok.getEndLocation(),
                              CommandTok.getCommandName(),
                              ArgTok.getLocation(),
                              ArgTok.getEndLocation(),
                              ArgTok.getText());
  } else {
    IC = S.actOnInlineCommand(CommandTok.getLocation(),
                              CommandTok.getEndLocation(),
                              CommandTok.getCommandName());
  }

  Retokenizer.putBackLeftoverTokens();

  return IC;
}

HTMLStartTagComment *Parser::parseHTMLStartTag() {
  assert(Tok.is(tok::html_start_tag));
  HTMLStartTagComment *HST =
      S.actOnHTMLStartTagStart(Tok.getLocation(),
                               Tok.getHTMLTagStartName());
  consumeToken();

  SmallVector<HTMLStartTagComment::Attribute, 2> Attrs;
  while (true) {
    switch (Tok.getKind()) {
    case tok::html_ident: {
      Token Ident = Tok;
      consumeToken();
      if (Tok.isNot(tok::html_equals)) {
        Attrs.push_back(HTMLStartTagComment::Attribute(Ident.getLocation(),
                                                       Ident.getHTMLIdent()));
        continue;
      }
      Token Equals = Tok;
      consumeToken();
      if (Tok.isNot(tok::html_quoted_string)) {
        Diag(Tok.getLocation(),
             diag::warn_doc_html_start_tag_expected_quoted_string)
          << SourceRange(Equals.getLocation());
        Attrs.push_back(HTMLStartTagComment::Attribute(Ident.getLocation(),
                                                       Ident.getHTMLIdent()));
        while (Tok.is(tok::html_equals) ||
               Tok.is(tok::html_quoted_string))
          consumeToken();
        continue;
      }
      Attrs.push_back(HTMLStartTagComment::Attribute(
                              Ident.getLocation(),
                              Ident.getHTMLIdent(),
                              Equals.getLocation(),
                              SourceRange(Tok.getLocation(),
                                          Tok.getEndLocation()),
                              Tok.getHTMLQuotedString()));
      consumeToken();
      continue;
    }

    case tok::html_greater:
      HST = S.actOnHTMLStartTagFinish(HST,
                                      copyArray(llvm::makeArrayRef(Attrs)),
                                      Tok.getLocation(),
                                      /* IsSelfClosing = */ false);
      consumeToken();
      return HST;

    case tok::html_slash_greater:
      HST = S.actOnHTMLStartTagFinish(HST,
                                      copyArray(llvm::makeArrayRef(Attrs)),
                                      Tok.getLocation(),
                                      /* IsSelfClosing = */ true);
      consumeToken();
      return HST;

    case tok::html_equals:
    case tok::html_quoted_string:
      Diag(Tok.getLocation(),
           diag::warn_doc_html_start_tag_expected_ident_or_greater);
      while (Tok.is(tok::html_equals) ||
             Tok.is(tok::html_quoted_string))
        consumeToken();
      if (Tok.is(tok::html_ident) ||
          Tok.is(tok::html_greater) ||
          Tok.is(tok::html_slash_greater))
        continue;

      return S.actOnHTMLStartTagFinish(HST,
                                       copyArray(llvm::makeArrayRef(Attrs)),
                                       SourceLocation(),
                                       /* IsSelfClosing = */ false);

    default:
      // Not a token from an HTML start tag.  Thus HTML tag prematurely ended.
      HST = S.actOnHTMLStartTagFinish(HST,
                                      copyArray(llvm::makeArrayRef(Attrs)),
                                      SourceLocation(),
                                      /* IsSelfClosing = */ false);
      bool StartLineInvalid;
      const unsigned StartLine = SourceMgr.getPresumedLineNumber(
                                                  HST->getLocation(),
                                                  &StartLineInvalid);
      bool EndLineInvalid;
      const unsigned EndLine = SourceMgr.getPresumedLineNumber(
                                                  Tok.getLocation(),
                                                  &EndLineInvalid);
      if (StartLineInvalid || EndLineInvalid || StartLine == EndLine)
        Diag(Tok.getLocation(),
             diag::warn_doc_html_start_tag_expected_ident_or_greater)
          << HST->getSourceRange();
      else {
        Diag(Tok.getLocation(),
             diag::warn_doc_html_start_tag_expected_ident_or_greater);
        Diag(HST->getLocation(), diag::note_doc_html_tag_started_here)
          << HST->getSourceRange();
      }
      return HST;
    }
  }
}

HTMLEndTagComment *Parser::parseHTMLEndTag() {
  assert(Tok.is(tok::html_end_tag));
  Token TokEndTag = Tok;
  consumeToken();
  SourceLocation Loc;
  if (Tok.is(tok::html_greater)) {
    Loc = Tok.getLocation();
    consumeToken();
  }

  return S.actOnHTMLEndTag(TokEndTag.getLocation(),
                           Loc,
                           TokEndTag.getHTMLTagEndName());
}

BlockContentComment *Parser::parseParagraphOrBlockCommand() {
  SmallVector<InlineContentComment *, 8> Content;

  while (true) {
    switch (Tok.getKind()) {
    case tok::verbatim_block_begin:
    case tok::verbatim_line_name:
    case tok::eof:
      assert(Content.size() != 0);
      break; // Block content or EOF ahead, finish this parapgaph.

    case tok::command:
      if (S.isBlockCommand(Tok.getCommandName())) {
        if (Content.size() == 0)
          return parseBlockCommand();
        break; // Block command ahead, finish this parapgaph.
      }
      if (S.isInlineCommand(Tok.getCommandName())) {
        Content.push_back(parseInlineCommand());
        continue;
      }

      // Not a block command, not an inline command ==> an unknown command.
      Content.push_back(S.actOnUnknownCommand(Tok.getLocation(),
                                              Tok.getEndLocation(),
                                              Tok.getCommandName()));
      consumeToken();
      continue;

    case tok::newline: {
      consumeToken();
      if (Tok.is(tok::newline) || Tok.is(tok::eof)) {
        consumeToken();
        break; // Two newlines -- end of paragraph.
      }
      if (Content.size() > 0)
        Content.back()->addTrailingNewline();
      continue;
    }

    // Don't deal with HTML tag soup now.
    case tok::html_start_tag:
      Content.push_back(parseHTMLStartTag());
      continue;

    case tok::html_end_tag:
      Content.push_back(parseHTMLEndTag());
      continue;

    case tok::text:
      Content.push_back(S.actOnText(Tok.getLocation(),
                                    Tok.getEndLocation(),
                                    Tok.getText()));
      consumeToken();
      continue;

    case tok::verbatim_block_line:
    case tok::verbatim_block_end:
    case tok::verbatim_line_text:
    case tok::html_ident:
    case tok::html_equals:
    case tok::html_quoted_string:
    case tok::html_greater:
    case tok::html_slash_greater:
      llvm_unreachable("should not see this token");
    }
    break;
  }

  return S.actOnParagraphComment(copyArray(llvm::makeArrayRef(Content)));
}

VerbatimBlockComment *Parser::parseVerbatimBlock() {
  assert(Tok.is(tok::verbatim_block_begin));

  VerbatimBlockComment *VB =
      S.actOnVerbatimBlockStart(Tok.getLocation(),
                                Tok.getVerbatimBlockName());
  consumeToken();

  // Don't create an empty line if verbatim opening command is followed
  // by a newline.
  if (Tok.is(tok::newline))
    consumeToken();

  SmallVector<VerbatimBlockLineComment *, 8> Lines;
  while (Tok.is(tok::verbatim_block_line) ||
         Tok.is(tok::newline)) {
    VerbatimBlockLineComment *Line;
    if (Tok.is(tok::verbatim_block_line)) {
      Line = S.actOnVerbatimBlockLine(Tok.getLocation(),
                                      Tok.getVerbatimBlockText());
      consumeToken();
      if (Tok.is(tok::newline)) {
        consumeToken();
      }
    } else {
      // Empty line, just a tok::newline.
      Line = S.actOnVerbatimBlockLine(Tok.getLocation(), "");
      consumeToken();
    }
    Lines.push_back(Line);
  }

  if (Tok.is(tok::verbatim_block_end)) {
    VB = S.actOnVerbatimBlockFinish(VB, Tok.getLocation(),
                                    Tok.getVerbatimBlockName(),
                                    copyArray(llvm::makeArrayRef(Lines)));
    consumeToken();
  } else {
    // Unterminated \\verbatim block
    VB = S.actOnVerbatimBlockFinish(VB, SourceLocation(), "",
                                    copyArray(llvm::makeArrayRef(Lines)));
  }

  return VB;
}

VerbatimLineComment *Parser::parseVerbatimLine() {
  assert(Tok.is(tok::verbatim_line_name));

  Token NameTok = Tok;
  consumeToken();

  SourceLocation TextBegin;
  StringRef Text;
  // Next token might not be a tok::verbatim_line_text if verbatim line
  // starting command comes just before a newline or comment end.
  if (Tok.is(tok::verbatim_line_text)) {
    TextBegin = Tok.getLocation();
    Text = Tok.getVerbatimLineText();
  } else {
    TextBegin = NameTok.getEndLocation();
    Text = "";
  }

  VerbatimLineComment *VL = S.actOnVerbatimLine(NameTok.getLocation(),
                                                NameTok.getVerbatimLineName(),
                                                TextBegin,
                                                Text);
  consumeToken();
  return VL;
}

BlockContentComment *Parser::parseBlockContent() {
  switch (Tok.getKind()) {
  case tok::text:
  case tok::command:
  case tok::html_start_tag:
  case tok::html_end_tag:
    return parseParagraphOrBlockCommand();

  case tok::verbatim_block_begin:
    return parseVerbatimBlock();

  case tok::verbatim_line_name:
    return parseVerbatimLine();

  case tok::eof:
  case tok::newline:
  case tok::verbatim_block_line:
  case tok::verbatim_block_end:
  case tok::verbatim_line_text:
  case tok::html_ident:
  case tok::html_equals:
  case tok::html_quoted_string:
  case tok::html_greater:
  case tok::html_slash_greater:
    llvm_unreachable("should not see this token");
  }
  llvm_unreachable("bogus token kind");
}

FullComment *Parser::parseFullComment() {
  // Skip newlines at the beginning of the comment.
  while (Tok.is(tok::newline))
    consumeToken();

  SmallVector<BlockContentComment *, 8> Blocks;
  while (Tok.isNot(tok::eof)) {
    Blocks.push_back(parseBlockContent());

    // Skip extra newlines after paragraph end.
    while (Tok.is(tok::newline))
      consumeToken();
  }
  return S.actOnFullComment(copyArray(llvm::makeArrayRef(Blocks)));
}

} // end namespace comments
} // end namespace clang
