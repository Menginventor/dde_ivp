from setuptools import setup, find_packages

setup(
    name="dde_ivp",
    version="0.1.0",
    author="Dhamdhawach  Horsuwan",
    author_email="meng.inventor@gmail.com",
    description="A package for solving delay differential equations",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "matplotlib",
        "scipy",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
)
