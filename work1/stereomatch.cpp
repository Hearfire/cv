#include"stereomatch.h"


//sum of absolute differences
Mat SAD::computerSAD(Mat& L, Mat& R) {
	int Height = L.rows;
	int Width = L.cols;
	Mat Kernel_L(Size(winSize, winSize), CV_8U, Scalar::all(0));
	Mat Kernel_R(Size(winSize, winSize), CV_8U, Scalar::all(0));
	Mat Disparity(Height, Width, CV_8U, Scalar(0)); //Parallax

    double complete = 0.1;

    //In fact ,i begin at i=DSR,as the left edge area of the left image usually doesn't match
	for (int i = 0; i < Width - winSize; i++)  
	{
		for (int j = 0; j < Height - winSize; j++)
		{
			Kernel_L = L(Rect(i, j, winSize, winSize));
			Mat MM(1, DSR, CV_32F, Scalar(0)); //

			for (int k = 0; k < DSR; k++)
			{
                //Calculate the sum of the absolute values of the difference between pixels in the kernel
				int x = i - k;
				if (x >= 0)
				{
					Kernel_R = R(Rect(x, j, winSize, winSize));
					Mat Dif;
					absdiff(Kernel_L, Kernel_R, Dif);
					Scalar ADD = sum(Dif);
					float a = ADD[0];
					MM.at<float>(k) = a;
				}

			}
            //The pixel with the lowest SAD value is the best match
			Point minLoc;
			minMaxLoc(MM, NULL, NULL, &minLoc, NULL);

			int loc = minLoc.x;//Parallax value

			Disparity.at<char>(j, i) = loc * 16;

		}
        if (i > complete* Width) {
            cout << complete * 100 << "% ";
            complete += 0.1;
        }
	}
    cout <<"100%"<< endl;
	return Disparity;
}


// transform 3 chanels image to grey image
Mat bgr_to_grey(const Mat& bgr)
{
    int width = bgr.size().width;
    int height = bgr.size().height;
    Mat grey(height, width, 0);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            uchar r = 0.333 * bgr.at<Vec3b>(y, x)[2];
            uchar g = 0.333 * bgr.at<Vec3b>(y, x)[1];
            uchar b = 0.333 * bgr.at<Vec3b>(y, x)[0];
            grey.at<uchar>(y, x) = uchar(r + g + b);
        }
    }

    return grey;
}
Mat NCC::computerNCC(Mat in1, Mat in2, bool add_constant = false)
{
    int width = in1.size().width;
    int height = in1.size().height;

    Mat left = bgr_to_grey(in1);
    Mat right = bgr_to_grey(in2);

    if (add_constant)
    {
        right += 10;
    }

    Mat depth(height, width, 0);
    vector< vector<double> > max_ncc; // store max NCC value

    for (int i = 0; i < height; ++i)
    {
        vector<double> tmp(width, -2);
        max_ncc.push_back(tmp);
    }

    double complete = 0.1;
    for (int offset = 1; offset <= max_offset; offset++)
    {
        Mat tmp(height, width, 0);

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < offset; x++)
            {
                tmp.at<uchar>(y, x) = right.at<uchar>(y, x);
            }

            for (int x = offset; x < width; x++)
            {
                tmp.at<uchar>(y, x) = right.at<uchar>(y, x - offset);
            }
        }

        // calculate each pixel's NCC value
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int start_x = max(0, x - kernel_size);
                int start_y = max(0, y - kernel_size);
                int end_x = min(width - 1, x + kernel_size);
                int end_y = min(height - 1, y + kernel_size);
                double n = (end_y - start_y) * (end_x - start_x);
                double res_ncc = 0;


                double left_mean = 0, right_mean = 0;
                double left_std = 0, right_std = 0;
                double numerator = 0;

                for (int i = start_y; i <= end_y; i++)
                {
                    for (int j = start_x; j <= end_x; j++)
                    {
                        left_mean += left.at<uchar>(i, j);
                        right_mean += tmp.at<uchar>(i, j);
                    }
                }

                left_mean /= n;
                right_mean /= n;

                for (int i = start_y; i <= end_y; i++)
                {
                    for (int j = start_x; j <= end_x; j++)
                    {
                        left_std += pow(left.at<uchar>(i, j) - left_mean, 2);
                        right_std += pow(tmp.at<uchar>(i, j) - right_mean, 2);
                        numerator += (left.at<uchar>(i, j) - left_mean) * (tmp.at<uchar>(i, j) - right_mean);
                    }
                }

                numerator /= n;
                left_std /= n;
                right_std /= n;
                res_ncc = numerator / (sqrt(left_std) * sqrt(right_std)) / n;


                // greater NCC value found
                if (res_ncc > max_ncc[y][x])
                {
                    max_ncc[y][x] = res_ncc;
                    // for better visualization
                    depth.at<uchar>(y, x) = (uchar)(offset * 3);
                }
            }
        }

        if (offset > complete * max_offset) {
            cout << complete * 100 << "% ";
            complete += 0.1;
        }
    }

    cout  << endl;

    return depth;
}
