debug版的exe路径 ：./x64/Debug/parseFBX.exe
源码路径：./parseFBX

通过命令行启动，需要传入参数：待解析的fbx文件（全称带拓展名） 输出XML文件名（全称带拓展名）
示例：（在 “./x64/Debug/”路径下）parseFBX.exe fbx_runtime_import_test.FBX result.xml
xml文件将会存储材质，shader相关信息，可以通过改变源码修改xml中信息。
如果需要查看fbx信息，可以加入>xxx.txt 将控制台信息写入txt文件中。
示例：（在 “./x64/Debug/”路径下）parseFBX.exe>out.txt fbx_runtime_import_test.FBX result.xml
控制台信息包含fbx解析的各种信息（目前只打开了材质相关信息）,可以通过修改源码改变打印消息。

PS：fbx文件可以放置和exe目录下，放置在其他位置时，命令行需要加入相对路径否则无法找到。