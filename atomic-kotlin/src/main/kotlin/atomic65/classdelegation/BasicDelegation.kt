package org.example.atomic65.classdelegation

interface AI
class A : AI
class B(val a: A) : AI by a

