#pragma once

#include "CaliStruct.h"
#define _CRT_SECURE_NO_WARNINGS
#ifndef FALSE
#define FALSE 0
#endif /* !FALSE */
#ifndef TRUE
#define TRUE 1
#endif /* !TRUE */

/*----------------------------------------------------------------------------*/
/* fatal error, print a message to standard error and exit
*/
/* static */ void error(const char *msg)
{
	fprintf(stderr, "error: %s\n", msg);
	exit(EXIT_FAILURE);
}

/*----------------------------------------------------------------------------*/
/* memory allocation, print an error and exit if fail
*/
// /* static */ void * xmalloc(size_t size)
// {
// 	void * p;
// 	if (size == 0) error("xmalloc input: zero size");
// 	p = malloc(size);
// 	if (p == NULL) error("out of memory");
// 	return p;
// }

/* open file, print an error and exit if fail
*/
/* static */ FILE *xfopen(const char *path, const char *mode)
{
	FILE *f = fopen(path, mode);
	if (f == NULL)
	{
		fprintf(stderr, "error: unable to open file '%s'\n", path);
		exit(EXIT_FAILURE);
	}
	return f;
}

/* close file, print an error and exit if fail
*/
/* static */ int xfclose(FILE *f)
{
	if (fclose(f) == EOF)
		error("unable to close file");
	return 0;
}

/* skip white characters and comments in a PGM file
*/
/* static */ void skip_whites_and_comments(FILE *f)
{
	int c;
	do
	{
		while (isspace(c = getc(f)))
			;		  /* skip spaces */
		if (c == '#') /* skip comments */
			while (c != '\n' && c != '\r' && c != EOF)
				c = getc(f);
	} while (c == '#' || isspace(c));
	if (c != EOF && ungetc(c, f) == EOF)
		error("unable to 'ungetc' while reading PGM file.");
}

/* read a number in ASCII from a PGM file
*/
/* static */ int get_num(FILE *f)
{
	int num, c;

	while (isspace(c = getc(f)))
		;
	if (!isdigit(c))
		error("corrupted PGM or PPM file.");
	num = c - '0';
	while (isdigit(c = getc(f)))
		num = 10 * num + c - '0';
	if (c != EOF && ungetc(c, f) == EOF)
		error("unable to 'ungetc' while reading PGM file.");

	return num;
}

/* memory allocation, print an error and exit if fail*/
void *xmalloc(size_t size)
{
	void *p;
	if (size == 0)
		error((char *)"xmalloc: zero size");
	p = malloc(size);
	if (p == NULL)
		error((char *)"xmalloc: out of memory");
	return p;
}

/* read a PGM image file
*/
double *read_pgm_image(char *name, int *X, int *Y)
{
	FILE *f;
	int i, n, depth, bin = FALSE;
	double *image;

	/* open file */
	f = xfopen(name, "rb"); /* open to read as a binary file (b option). otherwise,
							in some systems, it may behave differently */

	/* read header */
	if (getc(f) != 'P')
		error("not a PGM file!");
	if ((n = getc(f)) == '2')
		bin = FALSE;
	else if (n == '5')
		bin = TRUE;
	else
		error("not a PGM file!");
	skip_whites_and_comments(f);
	*X = get_num(f); /* X size */
	skip_whites_and_comments(f);
	*Y = get_num(f); /* Y size */
	skip_whites_and_comments(f);
	depth = get_num(f); /* pixel depth */
	if (depth < 0)
		error("pixel depth < 0, unrecognized PGM file");
	if (bin && depth > 255)
		error("pixel depth > 255, unrecognized PGM file");
	/* white before data */
	if (!isspace(getc(f)))
		error("corrupted PGM file.");

	/* get memory */
	image = (double *)xmalloc(*X * *Y * sizeof(double));

	/* read data */
	for (i = 0; i < (*X * *Y); i++)
		image[i] = (double)(bin ? getc(f) : get_num(f));

	/* close file */
	xfclose(f);

	/* return image */
	return image;
}

/*----------------------------------------------------------------------------*/
/* read a 2D ASC format file
*/
double *read_asc_file(char *name, int *X, int *Y)
{
	FILE *f;
	int i, n, Z, C;
	double val;
	double *image;

	char line[512];
	ifstream myfile(name);
	myfile.getline(line, 512);
	// /* open file */
	// f = xfopen(name, "rb"); /* open to read as a binary file (b option). otherwise,in some systems, it may behave differently */
	// /* read header */
	// n = fscanf_s(f, "%d%*c%d%*c%d%*c%d", X, Y, &Z, &C);
	istringstream iss(line);
	iss >> *X >> *Y >> Z >> C;

	if (n != 4 || *X <= 0 || *Y <= 0 || Z <= 0 || C <= 0)
		error("invalid ASC file");

	/* only gray level images are handled */
	if (Z != 1 || C != 1)
		error("only single channel ASC files are handled");

	/* get memory */
	image = (double *)xmalloc(*X * *Y * Z * C * sizeof(double));

	/* read data */
	for (i = 0; i < (*X * *Y * Z * C); i++)
	{
		myfile.getline(line, 512);
		iss.clear();
		iss >> val;
		//n = fscanf_s(f, "%lf", &val);
		if (n != 1)
			error("invalid ASC file");
		image[i] = val;
	}

	/* close file */
	xfclose(f);

	return image;
}

/*----------------------------------------------------------------------------*/
/* read an image from a file in ASC or PGM formats
*/
double *read_image(char *name, int *X, int *Y)
{
	int n = (int)strlen(name);
	char *ext = name + n - 4;

	if (n >= 4 && (strcmp(ext, ".asc") == 0 || strcmp(ext, ".ASC") == 0))
		return read_asc_file(name, X, Y);

	return read_pgm_image(name, X, Y);
}

/*----------------------------------------------------------------------------*/
/* write curves into a PDF file. the output is PDF version 1.4 as described in
"PDF Reference, third edition" by Adobe Systems Incorporated, 2001
*/
void write_curves_pdf(double *x, double *y, int *curve_limits, int M,
					  char *filename, int X, int Y, double width)
{
	FILE *pdf;
	long start1, start2, start3, start4, start5, startxref, stream_len;
	int i, j, k;

	// std::cout << "************************************************" << std::endl;
	// std::cout << "Write curve: " << curve_limits << std::endl;
	// std::cout << "Write x: " << x << std::endl;
	// std::cout << "Write y: " << y << std::endl;
	// std::cout << "************************************************" << std::endl;

	/* check input */
	if (filename == NULL)
		error("invalid filename in write_curves_pdf");
	if (M > 0 && (x == NULL || y == NULL || curve_limits == NULL))
		error("invalid curves data in write_curves_pdf");
	if (X <= 0 || Y <= 0)
		error("invalid image size in write_curves_pdf");

	/* open file */
	pdf = xfopen(filename, "wb"); /* open to write as a binary file (b option).
								  otherwise, in some systems,
								  it may behave differently */

	/* PDF header */
	fprintf(pdf, "%%PDF-1.4\n");
	/* The following PDF comment contains characters with ASCII codes greater
	than 128. This helps to classify the file as containing 8-bit binary data.
	See "PDF Reference" p.63. */
	fprintf(pdf, "%%%c%c%c%c\n", 0xe2, 0xe3, 0xcf, 0xd3);

	/* Catalog, Pages and Page objects */
	start1 = ftell(pdf);
	fprintf(pdf, "1 0 obj\n<</Type /Catalog /Pages 2 0 R>>\n");
	fprintf(pdf, "endobj\n");
	start2 = ftell(pdf);
	fprintf(pdf, "2 0 obj\n<</Type /Pages /Kids [3 0 R] /Count 1 ");
	fprintf(pdf, "/Resources <<>> /MediaBox [0 0 %d %d]>>\nendobj\n", X, Y);
	start3 = ftell(pdf);
	fprintf(pdf, "3 0 obj\n");
	fprintf(pdf, "<</Type /Page /Parent 2 0 R /Contents 4 0 R>>\n");
	fprintf(pdf, "endobj\n");

	/* Contents object - graphic contents */
	start4 = ftell(pdf);
	fprintf(pdf, "4 0 obj\n<</Length 5 0 R>>\n"); /* indirect length in obj 5 */
	fprintf(pdf, "stream\n");
	stream_len = ftell(pdf);
	fprintf(pdf, "%.4f w\n", width); /* set line width */
	for (k = 0; k < M; k++)			 /* write curves */
	{
		/* an offset of 0.5,0.5 is added to point coordinates so that the
		drawing has the right positioning when superposed on the image
		drawn to the same size. in that case, pixels are drawn as squares
		of size 1,1 and the coordinate of the detected edge points are
		relative to the center of those squares. thus the 0.5, 0.5 offset.
		*/

		/* initate chain */
		i = curve_limits[k];
		fprintf(pdf, "%.4f %.4f m\n", x[i] + 0.5, Y - y[i] - 0.5); /* first point */

		/* add remaining points of the curve */
		for (j = i + 1; j < curve_limits[k + 1]; j++)
			fprintf(pdf, "%.4f %.4f l\n", x[j] + 0.5, Y - y[j] - 0.5);

		/* if the curve is closed, market as such */
		j = curve_limits[k + 1] - 1;
		if (x[i] == x[j] && y[i] == y[j])
			fprintf(pdf, "h\n");

		/* end curve - stroke! */
		fprintf(pdf, "S\n");
	}
	stream_len = ftell(pdf) - stream_len; /* store stream length */
	fprintf(pdf, "\r\nendstream\n");	  /* EOL must be CRLF before endstream */
	fprintf(pdf, "endobj\n");

	/* Contents' stream length object - the use of this indirect object
	for the stream length allows to generate the PDF file in a single
	pass, specifying the stream��s length only when its contents have
	been generated. See "PDF Reference" p.40. */
	start5 = ftell(pdf);
	fprintf(pdf, "5 0 obj\n%ld\nendobj\n", stream_len);

	/* PDF Cross-reference table */
	startxref = ftell(pdf);
	fprintf(pdf, "xref\n");
	fprintf(pdf, "0 6\n");
	fprintf(pdf, "0000000000 65535 f\r\n"); /* EOL must be CRLF in xref table */
	fprintf(pdf, "%010ld 00000 n\r\n", start1);
	fprintf(pdf, "%010ld 00000 n\r\n", start2);
	fprintf(pdf, "%010ld 00000 n\r\n", start3);
	fprintf(pdf, "%010ld 00000 n\r\n", start4);
	fprintf(pdf, "%010ld 00000 n\r\n", start5);

	/* PDF trailer */
	fprintf(pdf, "trailer <</Size 6 /Root 1 0 R>>\n");
	fprintf(pdf, "startxref\n");
	fprintf(pdf, "%ld\n", startxref);
	fprintf(pdf, "%%%%EOF\n");

	/* close file */
	xfclose(pdf);
}

/*----------------------------------------------------------------------------*/
/* write curves into a TXT file
*/
void write_curves_txt(double *x, double *y, int *curve_limits, int M,
					  char *filename)
{
	FILE *txt;
	int i, k;

	/* check input */
	if (filename == NULL)
		error("invalid filename in write_curves_txt");
	if (M > 0 && (x == NULL || y == NULL || curve_limits == NULL))
		error("invalid curves data in write_curves_txt");

	/* open file */
	txt = xfopen(filename, "wb"); /* open to write as a binary file (b option).
								  otherwise, in some systems,
								  it may behave differently */

	/* write curves */
	for (k = 0; k < M; k++) /* write curves */
	{
		for (i = curve_limits[k]; i < curve_limits[k + 1]; i++)
			fprintf(txt, "%g %g\n", x[i], y[i]);
		fprintf(txt, "-1 -1\n"); /* end of chain */
	}

	/* close file */
	xfclose(txt);
}

/*----------------------------------------------------------------------------*/
/* write curves into a SVG file
*/
void write_curves_svg(double *x, double *y, int *curve_limits, int M,
					  char *filename, int X, int Y, double width)
{
	FILE *svg;
	int i, k;

	/* check input */
	if (filename == NULL)
		error("invalid filename in write_curves_svg");
	if (M > 0 && (x == NULL || y == NULL || curve_limits == NULL))
		error("invalid curves data in write_curves_svg");
	if (X <= 0 || Y <= 0)
		error("invalid image size in write_curves_svg");

	/* open file */
	svg = xfopen(filename, "wb"); /* open to write as a binary file (b option).
								  otherwise, in some systems,
								  it may behave differently */

	/* write SVG header */
	fprintf(svg, "<?xml version=\"1.0\" standalone=\"no\"?>\n");
	fprintf(svg, "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n");
	fprintf(svg, " \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n");
	fprintf(svg, "<svg width=\"%dpx\" height=\"%dpx\" ", X, Y);
	fprintf(svg, "version=\"1.1\"\n xmlns=\"http://www.w3.org/2000/svg\" ");
	fprintf(svg, "xmlns:xlink=\"http://www.w3.org/1999/xlink\">\n");

	/* write curves */
	for (k = 0; k < M; k++) /* write curves */
	{
		fprintf(svg, "<polyline stroke-width=\"%g\" ", width);
		fprintf(svg, "fill=\"none\" stroke=\"black\" points=\"");
		for (i = curve_limits[k]; i < curve_limits[k + 1]; i++)
			fprintf(svg, "%g,%g ", x[i], y[i]);
		fprintf(svg, "\"/>\n"); /* end of chain */
	}

	/* close SVG file */
	fprintf(svg, "</svg>\n");
	xfclose(svg);
}

void get_file_names_from_dir(string dir_path, vector<string> &file_names, bool is_recursive = false)
{
	filesystem::path p = dir_path;
	if (!exists(p))
	{
		return;
	}
	auto begin = filesystem::recursive_directory_iterator(p); //获取文件系统迭代器
	auto end = filesystem::recursive_directory_iterator();	  //end迭代器
	for (auto it = begin; it != end; it++)
	{

		auto &entry = *it;
		if (filesystem::is_regular_file(entry))
		{
			file_names.push_back(path(entry));
		}
		else if (filesystem::is_directory(entry) && is_recursive)
		{
			get_file_names_from_dir(path(entry), file_names, is_recursive);
		}
	}
}

//（C++17）获取传入目录下（string dir_path）所有的后缀名为“asc”的文件的绝对路径，返回使用引用方式传出vector<string> &file_paths
void get_filepaths_from_dir(string dir_path, vector<string> &file_paths, string file_format = "asc", bool is_recursive = false)
{
	//获取当前目录下所有文件
	vector<string> file_names;

	get_file_names_from_dir(dir_path, file_names, is_recursive);
	vector<string> file_names_temp = vector<string>(file_names);
	//遍历当前目录下所有文件，如果文件名称以file_format结尾，则将其路径添加到vector<string> &file_paths中
	for (int i = 0; i < file_names_temp.size(); i++)
	{
		if (file_names_temp[i].substr(file_names_temp[i].size() - file_format.size(), file_format.size()) == file_format)
		{
			file_paths.push_back(file_names[i]);
		}
	}
}

//影像文件排序
void sortFiles(vector<string> &file_paths)
{
	auto func = [&](string &aPath, string &bPath) -> bool
	{
		int aPosDot = aPath.find_last_of('.');
		int aPosStroke = aPath.find_last_of('_');
		string aIndexStr = aPath.substr(aPosStroke + 1, aPosDot - aPosStroke - 1);
		int bPosDot = bPath.find_last_of('.');
		int bPosStroke = bPath.find_last_of('_');
		string bIndexStr = bPath.substr(bPosStroke + 1, bPosDot - bPosStroke - 1);
		return atoi(aIndexStr.c_str()) < atoi(bIndexStr.c_str());
	};
	sort(file_paths.begin(), file_paths.end(), func);
}

////删除文件夹
//void DeleteDir(const string& strDirPath)
//{
//    string strPathTmp = strDirPath + "\\*.*";
//    intptr_t lFile;
//    string sAddPath;
//    struct _finddata_t sDate;
//    lFile = _findfirst(strPathTmp.c_str(), &sDate);
//    if (lFile == -1)
//    {
//        printf("read file fail!\n");
//        return;
//    }
//
//    intptr_t hFile = 0;
//    while ((hFile = _findnext(lFile, &sDate)) != -1)
//    {
//        if (sDate.attrib == _A_SUBDIR)
//        {
//            sAddPath = strDirPath;
//            sAddPath += "/";
//            sAddPath += sDate.name;
//            if (string(sDate.name) == "." || string(sDate.name) == "..")
//            {
//                continue;
//            }
//            DeleteDir(sAddPath);
//        }
//        else
//        {
//            string strFile = strDirPath + "\\" + string(sDate.name);
//            remove(strFile.c_str());
//        }
//    }
//
//    _findclose(lFile);
//    //删除文件夹
//    _rmdir(strDirPath.c_str());
//}
//
////生成指定文件夹
//void CreateFolder(string folderPath)
//{
//	if (_access(folderPath.c_str(), 0) == -1)
//		_mkdir(folderPath.c_str());
//	else
//	{
//		DeleteDir(folderPath);
//		_mkdir(folderPath.c_str());
//	}
//}

//获取影像名称
void GetImgName(string imgPath, string &imgName)
{
	int pos1 = imgPath.find_last_of("//");
	int pos2 = imgPath.find_last_of(".");
	imgName = imgPath.substr(pos1 + 1, pos2 - 1 - pos1);
}

//单张输出圆形标定板检测影像
void WriteSingleDetectCircleBoardImg(string imgPath, Mat srcImg, CircleBoardPara bPara, DetectCircleBoard board)
{
	Mat img_with_pts;
	cvtColor(srcImg, img_with_pts, CV_GRAY2BGR);

	int halfHeight = (bPara.nHeight - 1) / 2;
	int halfWidth = (bPara.nWidth - 1) / 2;

	vector<vector<Point>> vCntr;
	vCntr.resize(1);
	for (int i = 0; i < board.pts.size(); i++)
	{
		for (int j = 0; j < board.pts[i].cntr.size(); j++)
		{
			vCntr[0].emplace_back(Point(cvRound(board.pts[i].cntr[j].x), cvRound(board.pts[i].cntr[j].y)));
		}

		float x = board.pts[i].fitImgCoor.x;
		float y = board.pts[i].fitImgCoor.y;
		int row = (board.pts[i].idx.y > -99) ? halfHeight + board.pts[i].idx.y : 0;
		if (row - colorCnt >= 0)
			row = row - colorCnt;

		line(img_with_pts, Point(x - 10, y), Point(x + 10, y), ptColor[row]);
		line(img_with_pts, Point(x, y - 10), Point(x, y + 10), ptColor[row]);
		drawContours(img_with_pts, vCntr, 0, ptColor[row]);

		if (board.pts[i].idx.y != unused)
		{
			string text = to_string(board.pts[i].idx.y) + "," + to_string(board.pts[i].idx.x);
			putText(img_with_pts, text, Point(x - 10, y - 10), FONT_HERSHEY_PLAIN, 1.2, ptColor[row]);
		}

		vCntr[0].clear();
	}

	imwrite(imgPath, img_with_pts);
}

//单张输出圆形标定板信息文本
void WriteSingleCircleBoardTxt(string txtPath, CircleBoardPara bPara, DetectCircleBoard board)
{
	ofstream outfile;
	outfile.open(txtPath, ios::out);
	assert(outfile.is_open());

	string oneRowTxt = "";
	oneRowTxt = "标定板个数: " + to_string(1);
	outfile << oneRowTxt << endl;

	oneRowTxt = "行数: " + to_string(bPara.nHeight) + " 列数: " + to_string(bPara.nWidth);
	outfile << oneRowTxt << endl;

	int ptCnt = board.pts.size();
	oneRowTxt = "点数: " + to_string(ptCnt);
	outfile << oneRowTxt << endl;

	oneRowTxt = "**** 行号 **** 列号 **** 影像X坐标 **** 影像Y坐标 **** 三维X坐标 **** 三维Y坐标 **** 三维Z坐标 **** 圆直径（像素）****";
	outfile << oneRowTxt << endl;

	Point2f imgpt;
	Point2i imgIdx;
	Point3f objCoor;

	for (int i = 0; i < ptCnt; i++)
	{
		imgIdx = board.pts[i].idx;
		imgpt = board.pts[i].fitImgCoor;
		objCoor.x = (imgIdx.x <= unused) ? unused : imgIdx.x * bPara.nInterSize;
		objCoor.y = (imgIdx.y <= unused) ? unused : imgIdx.y * bPara.nInterSize;
		objCoor.z = 0.0;

		oneRowTxt = to_string(imgIdx.y) + "\t" + to_string(imgIdx.x) + "\t";
		oneRowTxt += to_string(imgpt.x) + "\t";
		oneRowTxt += to_string(imgpt.y) + "\t";
		oneRowTxt += to_string(objCoor.x) + "\t";
		oneRowTxt += to_string(objCoor.y) + "\t";
		oneRowTxt += to_string(objCoor.z) + "\t";
		oneRowTxt += to_string(board.pts[i].size) + "\t";
		oneRowTxt += to_string(board.pts[i].rrcEllipse.size.width) + "\t";
		oneRowTxt += to_string(board.pts[i].rrcEllipse.size.height);

		outfile << oneRowTxt << endl;
	}
	outfile << endl;

	oneRowTxt = "**** 行号 **** 列号 **** 轮廓点数 **** 轮廓X坐标 **** 轮廓Y坐标";
	outfile << oneRowTxt << endl;
	for (int i = 0; i < ptCnt; i++)
	{
		int cntrPtCnt = board.pts[i].cntr.size();
		imgIdx = board.pts[i].idx;

		oneRowTxt = to_string(imgIdx.y) + "\t" + to_string(imgIdx.x) + "\t";
		outfile << oneRowTxt << endl;

		oneRowTxt = to_string(cntrPtCnt);
		outfile << oneRowTxt << endl;

		for (int j = 0; j < cntrPtCnt; j++)
		{
			Point2f pt = board.pts[i].cntr[j];
			oneRowTxt = to_string(pt.x) + "\t" + to_string(pt.y);
			outfile << oneRowTxt << endl;
		}

		outfile << endl;
	}

	oneRowTxt = "**** the end ****";
	outfile << oneRowTxt << endl;

	outfile.close();
}

vector<string> split(const string &str, const string &delim)
{
	vector<string> res;
	if ("" == str)
		return res;
	//先将要切割的字符串从string类型转换为char*类型
	char *strs = new char[str.length() + 1]; //不要忘了
	strcpy(strs, str.c_str());

	char *d = new char[delim.length() + 1];
	strcpy(d, delim.c_str());

	char *p = strtok(strs, d);
	while (p)
	{
		string s = p;	  //分割得到的字符串转换为string类型
		res.push_back(s); //存入结果数组
		p = strtok(NULL, d);
	}

	return res;
}

void ReadSingleCircleBoardTxt(string txtPath, DetectCircleBoard &board)
{
	ifstream infile;
	infile.open(txtPath, ios::in);
	assert(infile.is_open());

	string oneRowTxt = "";
	//oneRowTxt = "标定板个数: " + to_string(1);
	//oneRowTxt = "行数: " + to_string(bPara.nHeight) + " 列数: " + to_string(bPara.nWidth);
	getline(infile, oneRowTxt);
	getline(infile, oneRowTxt);

	getline(infile, oneRowTxt);
	int pos = oneRowTxt.find(" ");
	string sub = oneRowTxt.substr(pos + 1, oneRowTxt.length() - pos);
	int ptCnt = atoi(sub.c_str());

	//oneRowTxt = "**** 行号 **** 列号 **** 影像X坐标 **** 影像Y坐标 **** 三维X坐标 **** 三维Y坐标 **** 三维Z坐标 **** 圆直径（像素）****";
	getline(infile, oneRowTxt);

	Point2f imgpt;
	Point2i imgIdx;
	Point3f objCoor;
	vector<string> subs;
	CirclePoint cpt;

	for (int i = 0; i < ptCnt; i++)
	{
		getline(infile, oneRowTxt);
		subs = split(oneRowTxt, "\t");

		cpt.idx = Point2i(stoi(subs[1]), stoi(subs[0]));
		cpt.fitImgCoor = Point2f(stod(subs[2]), stod(subs[3]));
		cpt.rrcEllipse = RotatedRect(cpt.fitImgCoor, Size(stod(subs[8]), stod(subs[9])), 0.0);
		cpt.size = stod(subs[7]);
		board.pts.push_back(cpt);
	}

	getline(infile, oneRowTxt);
	getline(infile, oneRowTxt);
	int cntrPtCnt = 0;
	//oneRowTxt = "**** 行号 **** 列号 **** 轮廓点数 **** 轮廓X坐标 **** 轮廓Y坐标";
	for (int i = 0; i < ptCnt; i++)
	{
		imgIdx = board.pts[i].idx;
		getline(infile, oneRowTxt);
		getline(infile, oneRowTxt);
		cntrPtCnt = atoi(oneRowTxt.c_str());
		//oneRowTxt = to_string(cntrPtCnt);

		for (int j = 0; j < cntrPtCnt; j++)
		{
			getline(infile, oneRowTxt);
			subs = split(oneRowTxt, "\t");
			board.pts[i].cntr.push_back(Point2f(stod(subs[0]), stod(subs[1])));
			//oneRowTxt = to_string(pt.x) + "\t" + to_string(pt.y);
		}

		getline(infile, oneRowTxt);
	}

	infile.close();
}

//输出单相机检校结果和误差
void OutputSingleCalibResult(string filePath, CircleBoardPara bPara, SingleCalibCamera camera, CalibImageData data)
{
	assert(camera.errData.camCnt == 1);

	ofstream outfile;
	outfile.open(filePath, ios::out);
	assert(outfile.is_open());
	string oneRowTxt = "";

	int camCnt = camera.errData.camCnt;
	oneRowTxt = "CamCnt: " + to_string(camCnt);
	outfile << oneRowTxt << endl;

	InnPara para;
	for (int i = 0; i < camCnt; i++)
	{
		oneRowTxt = "Camera " + to_string(i) + " CalibRMS: " + to_string(camera.errData.camRMS[i]);
		outfile << oneRowTxt << endl;

		camera.GetCameraPara(para);
		oneRowTxt = "Camera " + to_string(i) + " Paras:";
		oneRowTxt += to_string(para.fx) + "\t" + to_string(para.fy) + "\t" +
					 to_string(para.cx) + "\t" + to_string(para.cy) + "\t" + to_string(para.k1) + "\t" +
					 to_string(para.k2) + "\t" + to_string(para.k3) + "\t" + to_string(para.p1) + "\t" +
					 to_string(para.p2);
		outfile << oneRowTxt << endl;
	}

	oneRowTxt = "**** 行号 **** 列号 **** 影像X坐标 **** 影像Y坐标 **** 残差dX **** 残差dY **** 点位残差 **** ";
	outfile << oneRowTxt << endl;

	int imgCnt = data.objPts.size();
	int ptCnt = 0;
	double imgRms = 0.0;
	double tmpRes = 0.0;
	Point2f imgPt;
	Point2i imgIdx;
	Point3f objCoor;
	Point2f ptResidual;
	double fBoardInter = (double)bPara.nInterSize;
	double maxDX = DBL_MIN, maxDY = DBL_MIN, maxRes = DBL_MIN;
	double minDX = DBL_MAX, minDY = DBL_MAX, minRes = DBL_MAX;

	for (int j = 0; j < imgCnt; j++)
	{
		ptCnt = data.objPts[j].size();
		imgRms = camera.errData.imgRMS[0][j];
		oneRowTxt = "Cam " + to_string(camCnt - 1) + " Image " + to_string(j) +
					" PtCnt " + to_string(ptCnt) + " imgRMS " + to_string(imgRms);
		outfile << oneRowTxt << endl;
		Mat imgRes = camera.errData.imgResiduals[0][j];
		for (int k = 0; k < ptCnt; k++)
		{
			imgPt = data.imgPts[j][k];
			objCoor = data.objPts[j][k];
			imgIdx = Point2f(cvRound(objCoor.x / fBoardInter), cvRound(objCoor.y / fBoardInter));
			ptResidual = imgRes.at<Vec2f>(0, k);
			tmpRes = sqrt(ptResidual.x * ptResidual.x + ptResidual.y * ptResidual.y);

			oneRowTxt = to_string(imgIdx.y) + "\t" + to_string(imgIdx.x) + "\t";
			oneRowTxt += to_string(imgPt.x) + "\t" + to_string(imgPt.y) + "\t";
			oneRowTxt += to_string(ptResidual.x) + "\t" + to_string(ptResidual.y) + "\t";
			oneRowTxt += to_string(tmpRes) + "\t";
			outfile << oneRowTxt << endl;

			if (fabs(ptResidual.x) - maxDX > 0)
				maxDX = ptResidual.x;
			if (fabs(ptResidual.x) - minDX < 0)
				minDX = ptResidual.x;
			if (fabs(ptResidual.y) - maxDY > 0)
				maxDY = ptResidual.y;
			if (fabs(ptResidual.y) - minDY < 0)
				minDY = ptResidual.y;
			if (fabs(ptResidual.x) - maxDX > 0)
				maxDX = ptResidual.x;
			if (tmpRes - maxRes > 0)
				maxRes = tmpRes;
			if (tmpRes - minRes > 0)
				minRes = tmpRes;
		}
	}

	oneRowTxt = "**** **** **** ****";
	outfile << oneRowTxt << endl;

	oneRowTxt = "MaxDx: " + to_string(maxDX) + "\t MinDX: " + to_string(minDX);
	outfile << oneRowTxt << endl;

	oneRowTxt = "MaxDy: " + to_string(maxDY) + "\t MinDy: " + to_string(minDY);
	outfile << oneRowTxt << endl;

	oneRowTxt = "MaxRes: " + to_string(maxRes) + "\t MinRes: " + to_string(minRes);
	outfile << oneRowTxt << endl;

	oneRowTxt = "**** the end ****";
	outfile << oneRowTxt << endl;

	outfile.close();
}

//输出多相机检校结果和误差
void OutputMultiCalibResult(string filePath, MultiCalibCameras cameras, vector<CalibImageData> allData, CalibErrorData errors)
{
	assert(errors.camCnt > 1);

	ofstream outfile;
	outfile.open(filePath, ios::out);
	assert(outfile.is_open());
	string oneRowTxt = "";

	int camCnt = errors.camCnt;
	oneRowTxt = "CamCnt: " + to_string(camCnt);
	outfile << oneRowTxt << endl;

	InnPara para;
	for (int i = 0; i < camCnt; i++)
	{
		oneRowTxt = "Camera " + to_string(i) + " CalibRMS: " + to_string(errors.camRMS[i]);
		outfile << oneRowTxt << endl;

		cameras.GetCameraPara(i, para);
		oneRowTxt = "Camera " + to_string(i) + " Paras:";
		oneRowTxt += to_string(para.fx) + "\t" + to_string(para.fy) + "\t" +
					 to_string(para.cx) + "\t" + to_string(para.cy) + "\t" + to_string(para.k1) + "\t" +
					 to_string(para.k2) + "\t" + to_string(para.k3) + "\t" + to_string(para.p1) + "\t" +
					 to_string(para.p2);
		outfile << oneRowTxt << endl;
	}

	oneRowTxt = "**** the end ****";
	outfile << oneRowTxt << endl;

	outfile.close();
}