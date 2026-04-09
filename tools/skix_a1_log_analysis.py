#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets import QApplication
from log_analysis_core.main_window import LogAnalysisMainWindow

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion") # Good default style
    
    window = LogAnalysisMainWindow()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
