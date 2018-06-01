Dim WshShell

Set WshShell = WScript.CreateObject("WScript.Shell")

WshShell.Run "cmd /c "+WScript.Arguments.Item(1)+"\packages\realtek_ameba\sdk-ameba-v4.0b_without_NDA_GCC_V1.0.0\component\soc\realtek\8711b\misc\iar_utility\common\postbuild_img2.bat "+WScript.Arguments.Item(0)+" "+WScript.Arguments.Item(1), 0
