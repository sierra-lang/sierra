// RUN: %clang_cc1 -analyze -analyzer-checker=core,unix -verify %s

// Testing core functionality of the SValBuilder.

int SValBuilderLogicNoCrash(int *x) {
  return 3 - (int)(x +3);
}
