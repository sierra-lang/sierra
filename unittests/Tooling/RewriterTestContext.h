//===--- RewriterTestContext.h ----------------------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//  This file defines a utility class for Rewriter related tests.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_REWRITER_TEST_CONTEXT_H
#define LLVM_CLANG_REWRITER_TEST_CONTEXT_H

#include "clang/Basic/Diagnostic.h"
#include "clang/Basic/FileManager.h"
#include "clang/Basic/LangOptions.h"
#include "clang/Basic/SourceManager.h"
#include "clang/Frontend/DiagnosticOptions.h"
#include "clang/Frontend/TextDiagnosticPrinter.h"
#include "clang/Rewrite/Rewriter.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/raw_ostream.h"

namespace clang {

/// \brief A class that sets up a ready to use Rewriter.
///
/// Useful in unit tests that need a Rewriter. Creates all dependencies
/// of a Rewriter with default values for testing and provides convenience
/// methods, which help with writing tests that change files.
class RewriterTestContext {
 public:
  RewriterTestContext()
      : Diagnostics(llvm::IntrusiveRefCntPtr<DiagnosticIDs>()),
        DiagnosticPrinter(llvm::outs(), DiagnosticOptions()),
        Files((FileSystemOptions())),
        Sources(Diagnostics, Files),
        Rewrite(Sources, Options) {
    Diagnostics.setClient(&DiagnosticPrinter, false);
  }

  ~RewriterTestContext() {
    if (TemporaryDirectory.isValid()) {
      std::string ErrorInfo;
      TemporaryDirectory.eraseFromDisk(true, &ErrorInfo);
      assert(ErrorInfo.empty());
    }
  }

  FileID createInMemoryFile(StringRef Name, StringRef Content) {
    const llvm::MemoryBuffer *Source =
      llvm::MemoryBuffer::getMemBuffer(Content);
    const FileEntry *Entry =
      Files.getVirtualFile(Name, Source->getBufferSize(), 0);
    Sources.overrideFileContents(Entry, Source, true);
    assert(Entry != NULL);
    return Sources.createFileID(Entry, SourceLocation(), SrcMgr::C_User);
  }

  FileID createOnDiskFile(StringRef Name, StringRef Content) {
    if (!TemporaryDirectory.isValid()) {
      std::string ErrorInfo;
      TemporaryDirectory = llvm::sys::Path::GetTemporaryDirectory(&ErrorInfo);
      assert(ErrorInfo.empty());
    }
    llvm::SmallString<1024> Path(TemporaryDirectory.str());
    llvm::sys::path::append(Path, Name);
    std::string ErrorInfo;
    llvm::raw_fd_ostream OutStream(Path.c_str(),
                                   ErrorInfo, llvm::raw_fd_ostream::F_Binary);
    assert(ErrorInfo.empty());
    OutStream << Content;
    OutStream.close();
    const FileEntry *File = Files.getFile(Path);
    assert(File != NULL);
    return Sources.createFileID(File, SourceLocation(), SrcMgr::C_User);
  }

  SourceLocation getLocation(FileID ID, unsigned Line, unsigned Column) {
    SourceLocation Result = Sources.translateFileLineCol(
        Sources.getFileEntryForID(ID), Line, Column);
    assert(Result.isValid());
    return Result;
  }

  std::string getRewrittenText(FileID ID) {
    std::string Result;
    llvm::raw_string_ostream OS(Result);
    Rewrite.getEditBuffer(ID).write(OS);
    return Result;
  }

  std::string getFileContentFromDisk(StringRef Name) {
    llvm::SmallString<1024> Path(TemporaryDirectory.str());
    llvm::sys::path::append(Path, Name);
    // We need to read directly from the FileManager without relaying through
    // a FileEntry, as otherwise we'd read through an already opened file
    // descriptor, which might not see the changes made.
    // FIXME: Figure out whether there is a way to get the SourceManger to
    // reopen the file.
    return Files.getBufferForFile(Path, NULL)->getBuffer();
  }

  DiagnosticsEngine Diagnostics;
  TextDiagnosticPrinter DiagnosticPrinter;
  FileManager Files;
  SourceManager Sources;
  LangOptions Options;
  Rewriter Rewrite;

  // Will be set once on disk files are generated. 
  llvm::sys::Path TemporaryDirectory;
};

} // end namespace clang

#endif
