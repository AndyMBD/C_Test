@ECHO OFF
set PATH=%MLX_PYTHON_PATH%
python -m pip install "numpy>=1.19.0"
python -m pip install "matplotlib>=3.2.2"
python -m pip install "jupyter==1.0.0" "pywinpty>==0.5.5,<1.0.0"
python -m notebook generate_svm.ipynb
