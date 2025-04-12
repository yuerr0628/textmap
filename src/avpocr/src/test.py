from paddleocr import PaddleOCR, draw_ocr

# 'use_gpu=False'不用gpu，默认使用GPU
# 'use_angle_cls=True'自动下载相关的包
# 'lang="ch"'设置语言，支持中英文、英文、法语、德语、韩语、日语，参数依次为`ch`, `en`, `french`, `german`, `korean`, `japan`。
ocr = PaddleOCR( use_angle_cls=True, lang="ch")
img_path = '/mnt/pool/yhy/ocr_video/src/avpocr/src/sw.png'
result = ocr.ocr(img_path, cls=True)

# line是一个列表' [[文本框的位置],(文字,置信度)] '
for line in result:
    print(line)

