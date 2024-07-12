function agentpath = importfile(filename, dataLines)
%IMPORTFILE 从文本文件中导入数据
%  AGENT1PATH = IMPORTFILE(FILENAME)读取文本文件 FILENAME 中默认选定范围的数据。
%  以表形式返回数据。
%
%  AGENT1PATH = IMPORTFILE(FILE, DATALINES)按指定行间隔读取文本文件 FILENAME
%  中的数据。对于不连续的行间隔，请将 DATALINES 指定为正整数标量或 N×2 正整数标量数组。
%
%  示例:
%  agent1path = importfile("D:\工作文档\清华中期汇报材料\coverage_planning_with_MILP_2\coverage_planning_with_MILP\coverage_planning_with_MILP\agent1path.csv", [2, Inf]);
%
%  另请参阅 READTABLE。
%
% 由 MATLAB 于 2023-09-09 10:51:29 自动生成

%% 输入处理

% 如果不指定 dataLines，请定义默认范围
if nargin < 2
    dataLines = [2, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 3);

% 指定范围和分隔符
opts.DataLines = dataLines;
opts.Delimiter = ",";

% 指定列名称和类型
opts.VariableNames = ["Index", "Enu_x", "Enu_y"];
opts.VariableTypes = ["double", "double", "double"];

% 指定文件级属性
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% 导入数据
agentpath = readtable(filename, opts);

end