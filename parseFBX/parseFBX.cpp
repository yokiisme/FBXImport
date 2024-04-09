//// GenFBXBinary.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
////
//
#include <iostream>
#include <string>
#include <fbxsdk.h>
#include "Common.h"
#include "DLLInterface.h"


void main(int argc, char** argv)
{
	if (argc != 3 && argc != 4) return;
	char* str = argv[1];
	char* outpath = argv[2];
	char* paramater = nullptr;
	if(argc == 4)
		paramater = argv[3];
	ParseFBX(str, outpath, paramater);
}