#include "CammerWidget.h"
#include "ui_cammerwidget.h"

// Ĭ����ʾ֡��
// defult display frequency
#define DEFAULT_SHOW_RATE (30)
#define DEFAULT_ERROR_STRING ("N/A") 
#define MAX_FRAME_STAT_NUM (50) 
#define MIN_LEFT_LIST_NUM (2)

// List�ĸ���ʱ������һ֡��ʱ�������
// The maximum time interval between the update of list and the latest frame
#define MAX_STATISTIC_INTERVAL (5000000000)

// ȡ���ص�����
// get frame callback function
static void FrameCallback(IMV_Frame* pFrame, void* pUser)
{
	CammerWidget* pCammerWidget = (CammerWidget*)pUser;
	if (!pCammerWidget)
	{
		printf("pCammerWidget is NULL!\n");
		return;
	}

	CFrameInfo frameInfo;
	frameInfo.m_nWidth = (int)pFrame->frameInfo.width;
	frameInfo.m_nHeight = (int)pFrame->frameInfo.height;
	frameInfo.m_nBufferSize = (int)pFrame->frameInfo.size;
	frameInfo.m_nPaddingX = (int)pFrame->frameInfo.paddingX;
	frameInfo.m_nPaddingY = (int)pFrame->frameInfo.paddingY;
	frameInfo.m_ePixelType = pFrame->frameInfo.pixelFormat;
	frameInfo.m_pImageBuf = (unsigned char *)malloc(sizeof(unsigned char) * frameInfo.m_nBufferSize);
	frameInfo.m_nTimeStamp = pFrame->frameInfo.timeStamp;

	// �ڴ�����ʧ�ܣ�ֱ�ӷ���
	// memory application failed, return directly
	if (frameInfo.m_pImageBuf != NULL)
	{
		memcpy(frameInfo.m_pImageBuf, pFrame->pData, frameInfo.m_nBufferSize);

		if (pCammerWidget->m_qDisplayFrameQueue.size() > 16)
		{
			CFrameInfo frameOld;
			if (pCammerWidget->m_qDisplayFrameQueue.get(frameOld))
			{
				free(frameOld.m_pImageBuf);
				frameOld.m_pImageBuf = NULL;
			}
		}

		pCammerWidget->m_qDisplayFrameQueue.push_back(frameInfo);
	}

	pCammerWidget->recvNewFrame(pFrame->frameInfo.size);
}

// ��ʾ�߳�
// display thread
void* displayThread(void* pUser)
{
	CammerWidget* pCammerWidget = (CammerWidget*)pUser;
	if (!pCammerWidget)
	{
		printf("pCammerWidget is NULL!\n");
		return NULL;
	}

	pCammerWidget->DisplayThreadProc();

	return NULL;
}

CammerWidget::CammerWidget(QWidget *parent) :
    QWidget(parent)
    ,ui(new Ui::CammerWidget)
	, m_currentCameraKey("")
	, m_devHandle(NULL)
	, m_threadID(0)
	, m_isExitDisplayThread(false)
	, m_nDisplayInterval(0)
	, m_nFirstFrameTime(0)
	, m_nLastFrameTime(0)
	, m_nTotalFrameCount(0)
	, m_bNeedUpdate(true)
{
    ui->setupUi(this);

	qRegisterMetaType<IMV_EPixelType>("IMV_EPixelType");
	connect(this, SIGNAL(signalShowImage(unsigned char*, int, int, IMV_EPixelType)), this, SLOT(ShowImage(unsigned char*, int, int, IMV_EPixelType)));

	// Ĭ����ʾ30֡
	// defult display 30 frames 
	setDisplayFPS(DEFAULT_SHOW_RATE);

	m_elapsedTimer.start();

	// ������ʾ�߳�
	// start display thread
	pthread_create(&m_threadID, 0, displayThread, this);

	if (m_threadID == 0)
	{
		printf("Failed to create display thread!\n");
	}
	else
	{
		m_isExitDisplayThread = false;
	}
}

CammerWidget::~CammerWidget()
{
    // �ر���ʾ�߳�
	// close display thread
	m_isExitDisplayThread = true;
	pthread_join(m_threadID, NULL);

    delete ui;
}

// �����ع�
// set exposeTime
bool CammerWidget::SetExposeTime(double dExposureTime)
{
	if (!m_devHandle)
	{
		return false;
	}

	int ret = IMV_OK;

	ret = IMV_SetDoubleFeatureValue(m_devHandle, "ExposureTime", dExposureTime);
	if (IMV_OK != ret)
	{
		printf("set ExposureTime value = %0.2f fail, ErrorCode[%d]\n", dExposureTime, ret);
		return false;
	}

	return true;
}

// ��������
// set gain
bool CammerWidget::SetAdjustPlus(double dGainRaw)
{
	if (!m_devHandle)
	{
		return false;
	}

	int ret = IMV_OK;

	ret = IMV_SetDoubleFeatureValue(m_devHandle, "GainRaw", dGainRaw);
	if (IMV_OK != ret)
	{
		printf("set GainRaw value = %0.2f fail, ErrorCode[%d]\n", dGainRaw, ret);
		return false;
	}

	return true;
}

// �����
// open camera
bool CammerWidget::CameraOpen(void)
{
	int ret = IMV_OK;

	if (m_currentCameraKey.length() == 0)
	{
		printf("open camera fail. No camera.\n");
		return false;
	}

	if (m_devHandle)
	{
		printf("m_devHandle is already been create!\n");
		return false;
	}
	QByteArray cameraKeyArray = m_currentCameraKey.toLocal8Bit();
	char* cameraKey = cameraKeyArray.data();

	ret = IMV_CreateHandle(&m_devHandle, modeByCameraKey, (void*)cameraKey);
	if (IMV_OK != ret)
	{
		printf("create devHandle failed! cameraKey[%s], ErrorCode[%d]\n", cameraKey, ret);
		return false;
	}

	// ����� 
	// Open camera 
	ret = IMV_Open(m_devHandle);
	if (IMV_OK != ret)
	{
		printf("open camera failed! ErrorCode[%d]\n", ret);
		return false;
	}

	return true;
}

// �ر����
// close camera
bool CammerWidget::CameraClose(void)
{
	if (!m_devHandle)
	{
		return false;
	}

	int ret = IMV_OK;

	if (!m_devHandle)
	{
		printf("close camera fail. No camera.\n");
		return false;
	}

	if (false == IMV_IsOpen(m_devHandle))
	{
		printf("camera is already close.\n");
		return false;
	}

	ret = IMV_Close(m_devHandle);
	if (IMV_OK != ret)
	{
		printf("close camera failed! ErrorCode[%d]\n", ret);
		return false;
	}

	ret = IMV_DestroyHandle(m_devHandle);
	if (IMV_OK != ret)
	{
		printf("destroy devHandle failed! ErrorCode[%d]\n", ret);
		return false;
	}

	m_devHandle = NULL;

	return true;
}

// ��ʼ�ɼ�
// start grabbing
bool CammerWidget::CameraStart()
{
	if (!m_devHandle)
	{
		return false;
	}

	int ret = IMV_OK;

	if (IMV_IsGrabbing(m_devHandle))
	{
		printf("camera is already grebbing.\n");
		return false;
	}


	ret = IMV_AttachGrabbing(m_devHandle, FrameCallback, this);
	if (IMV_OK != ret)
	{
		printf("Attach grabbing failed! ErrorCode[%d]\n", ret);
		return false;
	}

	ret = IMV_StartGrabbing(m_devHandle);
	if (IMV_OK != ret)
	{
		printf("start grabbing failed! ErrorCode[%d]\n", ret);
		return false;
	}

	return true;
}

// ֹͣ�ɼ�
// stop grabbing
bool CammerWidget::CameraStop()
{
	if (!m_devHandle)
	{
		return false;
	}

	int ret = IMV_OK;
	if (!IMV_IsGrabbing(m_devHandle))
	{
		printf("camera is already stop grebbing.\n");
		return false;
	}

	ret = IMV_StopGrabbing(m_devHandle);
	if (IMV_OK != ret)
	{
		printf("Stop grabbing failed! ErrorCode[%d]\n", ret);
		return false;
	}

	// �����ʾ���� 
	// clear display queue
	CFrameInfo frameOld;
	while (m_qDisplayFrameQueue.get(frameOld))
	{
		free(frameOld.m_pImageBuf);
		frameOld.m_pImageBuf = NULL;
	}

	m_qDisplayFrameQueue.clear();

	return true;
}

// �л��ɼ���ʽ��������ʽ �������ɼ����ⲿ���������������
// Switch acquisition mode and triggering mode (continuous acquisition, external triggering and software triggering)
bool CammerWidget::CameraChangeTrig(ETrigType trigType)
{
	if (!m_devHandle)
	{
		return false;
	}

	int ret = IMV_OK;

	if (trigContinous == trigType)
	{
		// ���ô���ģʽ
		// set trigger mode
		ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "Off");
		if (IMV_OK != ret)
		{
			printf("set TriggerMode value = Off fail, ErrorCode[%d]\n", ret);
			return false;
		}
	}
	else if (trigSoftware == trigType)
	{
		// ���ô�����
		// set trigger
		ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSelector", "FrameStart");
		if (IMV_OK != ret)
		{
			printf("set TriggerSelector value = FrameStart fail, ErrorCode[%d]\n", ret);
			return false;
		}

		// ���ô���ģʽ
		// set trigger mode
		ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "On");
		if (IMV_OK != ret)
		{
			printf("set TriggerMode value = On fail, ErrorCode[%d]\n", ret);
			return false;
		}

		// ���ô���ԴΪ����
		// set triggerSource as software trigger
		ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSource", "Software");
		if (IMV_OK != ret)
		{
			printf("set TriggerSource value = Software fail, ErrorCode[%d]\n", ret);
			return false;
		}
	}
	else if (trigLine == trigType)
	{
		// ���ô�����
		// set trigger
		ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSelector", "FrameStart");
		if (IMV_OK != ret)
		{
			printf("set TriggerSelector value = FrameStart fail, ErrorCode[%d]\n", ret);
			return false;
		}

		// ���ô���ģʽ
		// set trigger mode
		ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "On");
		if (IMV_OK != ret)
		{
			printf("set TriggerMode value = On fail, ErrorCode[%d]\n", ret);
			return false;
		}

		// ���ô���ԴΪLine1����
		// set trigggerSource as Line1 trigger 
		ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSource", "Line1");
		if (IMV_OK != ret)
		{
			printf("set TriggerSource value = Line1 fail, ErrorCode[%d]\n", ret);
			return false;
		}
	}

	return true;
}

// ִ��һ������
// execute one software trigger
bool CammerWidget::ExecuteSoftTrig(void)
{
	if (!m_devHandle)
	{
		return false;
	}

	int ret = IMV_OK;

	ret = IMV_ExecuteCommandFeature(m_devHandle, "TriggerSoftware");
	if (IMV_OK != ret)
	{
		printf("ExecuteSoftTrig fail, ErrorCode[%d]\n", ret);
		return false;
	}

	printf("ExecuteSoftTrig success.\n");
	return true;
}

// ���õ�ǰ���  
// set current camera
void CammerWidget::SetCamera(const QString& strKey)
{
	m_currentCameraKey = strKey;
}

// ��ʾ
// diaplay
bool CammerWidget::ShowImage(unsigned char* pRgbFrameBuf, int nWidth, int nHeight, IMV_EPixelType nPixelFormat)
{
	QImage image;
	if (NULL == pRgbFrameBuf ||
		nWidth == 0 ||
		nHeight == 0)
	{
		printf("%s image is invalid.\n", __FUNCTION__);
		return false;
	}
    if (gvspPixelMono8 == nPixelFormat)
	{
		image = QImage(pRgbFrameBuf, nWidth, nHeight, QImage::Format_Grayscale8);
	}
	else
	{
		image = QImage(pRgbFrameBuf,nWidth, nHeight, QImage::Format_RGB888);
	}
    // ��QImage�Ĵ�С���������죬��label�Ĵ�С����һ�¡�����label������ʾ������ͼƬ
	// Shrink or stretch the size of Qimage to match the size of the label. In this way, the complete image can be displayed in the label
	QImage imageScale = image.scaled(QSize(ui->label_Pixmap->width(), ui->label_Pixmap->height()));
	QPixmap pixmap = QPixmap::fromImage(imageScale);
	ui->label_Pixmap->setPixmap(pixmap);
	free(pRgbFrameBuf);

	return true;
}

// ��ʾ�߳�
// display thread
void CammerWidget::DisplayThreadProc()
{
	while (!m_isExitDisplayThread)
	{
		CFrameInfo frameInfo;

		if (false == m_qDisplayFrameQueue.get(frameInfo))
		{
			usleep(1000);
			continue;
		}

		// �ж��Ƿ�Ҫ��ʾ��������ʾ���ޣ�30֡�����Ͳ���ת�롢��ʾ����
		// Judge whether to display. If the upper display limit (30 frames) is exceeded, transcoding and display processing will not be performed
		if (!isTimeToDisplay())
		{
			// �ͷ��ڴ�
			// release memory
			free(frameInfo.m_pImageBuf);
			continue;
		}

		// mono8��ʽ�ɲ���ת�룬ֱ����ʾ��������ʽ��Ҫ����ת�������ʾ 
		// mono8 format can be displayed directly without transcoding. Other formats can be displayed only after transcoding
        if (gvspPixelMono8 == frameInfo.m_ePixelType)
		{
            // ��ʾ�߳��з�����ʾ�źţ������߳�����ʾͼ��
			// Send display signal in display thread and display image in main thread
			emit signalShowImage(frameInfo.m_pImageBuf, frameInfo.m_nWidth, frameInfo.m_nHeight, frameInfo.m_ePixelType);
		}
		else
		{
			// ת��
			// transcoding
			unsigned char *pRGBbuffer = NULL;
			int nRgbBufferSize = frameInfo.m_nWidth * frameInfo.m_nHeight * 3;
			pRGBbuffer = (unsigned char *)malloc(nRgbBufferSize);
			if (pRGBbuffer == NULL)
			{
				// �ͷ��ڴ�
				// release memory
				free(frameInfo.m_pImageBuf);
				printf("RGBbuffer malloc failed.\n");
				continue;
			}

			IMV_PixelConvertParam stPixelConvertParam;
			stPixelConvertParam.nWidth = frameInfo.m_nWidth;
			stPixelConvertParam.nHeight = frameInfo.m_nHeight;
			stPixelConvertParam.ePixelFormat = frameInfo.m_ePixelType;
			stPixelConvertParam.pSrcData = frameInfo.m_pImageBuf;
			stPixelConvertParam.nSrcDataLen = frameInfo.m_nBufferSize;
			stPixelConvertParam.nPaddingX = frameInfo.m_nPaddingX;
			stPixelConvertParam.nPaddingY = frameInfo.m_nPaddingY;
			stPixelConvertParam.eBayerDemosaic = demosaicNearestNeighbor;
			stPixelConvertParam.eDstPixelFormat = gvspPixelRGB8;
			stPixelConvertParam.pDstBuf = pRGBbuffer;
			stPixelConvertParam.nDstBufSize = nRgbBufferSize;

			int ret = IMV_PixelConvert(m_devHandle, &stPixelConvertParam);
			if (IMV_OK != ret)
			{
				// �ͷ��ڴ� 
				// release memory
				printf("image convert to RGB8 failed! ErrorCode[%d]\n", ret);
				free(frameInfo.m_pImageBuf);
				free(pRGBbuffer);
				pRGBbuffer = NULL;
				continue;
			}

			// �ͷ��ڴ�
			// release memory
			free(frameInfo.m_pImageBuf);

            // ��ʾ�߳��з�����ʾ�źţ������߳�����ʾͼ��
			// Send display signal in display thread and display image in main thread
			emit signalShowImage(pRGBbuffer, (int)stPixelConvertParam.nWidth, (int)stPixelConvertParam.nHeight, stPixelConvertParam.eDstPixelFormat);

		}
	}
}

bool CammerWidget::isTimeToDisplay()
{
	m_mxTime.lock();

	// ����ʾ
	// don't display
	if (m_nDisplayInterval <= 0)
	{
		m_mxTime.unlock();
		return false;
	}

	// ��һ֡������ʾ
	// the frist frame must be displayed
	if (m_nFirstFrameTime == 0 || m_nLastFrameTime == 0)
	{
		m_nFirstFrameTime = m_elapsedTimer.nsecsElapsed();
		m_nLastFrameTime = m_nFirstFrameTime;
		m_mxTime.unlock();
		return true;
	}

	// ��ǰ֡����һ֡�ļ�����������ʾ�������ʾ
	// display if the interval between the current frame and the previous frame is greater than the display interval
	uint64_t nCurTimeTmp = m_elapsedTimer.nsecsElapsed();
	uint64_t nAcquisitionInterval = nCurTimeTmp - m_nLastFrameTime;
	if (nAcquisitionInterval > m_nDisplayInterval)
	{
		m_nLastFrameTime = nCurTimeTmp;
		m_mxTime.unlock();
		return true;
	}

	// ��ǰ֡����ڵ�һ֡��ʱ����
	// Time interval between the current frame and the first frame
	uint64_t nPre = (m_nLastFrameTime - m_nFirstFrameTime) % m_nDisplayInterval;
	if (nPre + nAcquisitionInterval > m_nDisplayInterval)
	{
		m_nLastFrameTime = nCurTimeTmp;
		m_mxTime.unlock();
		return true;
	}

	m_mxTime.unlock();
	return false;
}

// ������ʾƵ��
// set display frequency
void CammerWidget::setDisplayFPS(int nFPS)
{
	m_mxTime.lock();
	if (nFPS > 0)
	{
		m_nDisplayInterval = 1000 * 1000 * 1000 / nFPS;
	}
	else
	{
		m_nDisplayInterval = 0;
	}
	m_mxTime.unlock();
}

// ���ڹر���Ӧ����
// window close response function
void CammerWidget::closeEvent(QCloseEvent * event)
{
	if (event)
	{
		IMV_DestroyHandle(m_devHandle);
		m_devHandle = NULL;
	}
}

// ״̬��ͳ����Ϣ ��ʼ
// Status bar statistics begin
void CammerWidget::resetStatistic()
{
	QMutexLocker locker(&m_mxStatistic);
	m_nTotalFrameCount = 0;
	m_listFrameStatInfo.clear();
	m_bNeedUpdate = true;
}
QString CammerWidget::getStatistic()
{
	if (m_mxStatistic.tryLock(30))
	{
		if (m_bNeedUpdate)
		{
			updateStatistic();
		}

		m_mxStatistic.unlock();
		return m_strStatistic;
	}
	return "";
}
void CammerWidget::updateStatistic()
{
	size_t nFrameCount = m_listFrameStatInfo.size();
	QString strFPS = DEFAULT_ERROR_STRING;
	QString strSpeed = DEFAULT_ERROR_STRING;

	if (nFrameCount > 1)
	{
		quint64 nTotalSize = 0;
		FrameList::const_iterator it = m_listFrameStatInfo.begin();

		if (m_listFrameStatInfo.size() == 2)
		{
            nTotalSize = m_listFrameStatInfo.back().m_nFrameSize;
		}
		else
		{
			for (++it; it != m_listFrameStatInfo.end(); ++it)
			{
                nTotalSize += it->m_nFrameSize;
			}
		}

		const FrameStatInfo& first = m_listFrameStatInfo.front();
		const FrameStatInfo& last = m_listFrameStatInfo.back();

        qint64 nsecs = last.m_nPassTime - first.m_nPassTime;

		if (nsecs > 0)
		{
			double dFPS = (nFrameCount - 1) * ((double)1000000000.0 / nsecs);
			double dSpeed = nTotalSize * ((double)1000000000.0 / nsecs) / (1000.0) / (1000.0) * (8.0);
			strFPS = QString::number(dFPS, 'f', 2);
			strSpeed = QString::number(dSpeed, 'f', 2);
		}
	}

	m_strStatistic = QString("Stream: %1 images   %2 FPS   %3 Mbps")
		.arg(m_nTotalFrameCount)
		.arg(strFPS)
		.arg(strSpeed);
	m_bNeedUpdate = false;
}
void CammerWidget::recvNewFrame(unsigned int size)
{
	QMutexLocker locker(&m_mxStatistic);
	if (m_listFrameStatInfo.size() >= MAX_FRAME_STAT_NUM)
	{
		m_listFrameStatInfo.pop_front();
	}
	m_listFrameStatInfo.push_back(FrameStatInfo(size, m_elapsedTimer.nsecsElapsed()));
	++m_nTotalFrameCount;

	if (m_listFrameStatInfo.size() > MIN_LEFT_LIST_NUM)
	{
		FrameStatInfo infoFirst = m_listFrameStatInfo.front();
		FrameStatInfo infoLast = m_listFrameStatInfo.back();
        while (m_listFrameStatInfo.size() > MIN_LEFT_LIST_NUM && infoLast.m_nPassTime - infoFirst.m_nPassTime > MAX_STATISTIC_INTERVAL)
		{
			m_listFrameStatInfo.pop_front();
			infoFirst = m_listFrameStatInfo.front();
		}
	}

	m_bNeedUpdate = true;
}
// ״̬��ͳ����Ϣ end 
// Status bar statistics ending