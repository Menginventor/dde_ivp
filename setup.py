from setuptools import setup, find_packages

setup(
    name="dde_ivp",
    version="0.1.2",
    author="Dhamdhawach  Horsuwan",
    author_email="meng.inventor@gmail.com",
    description="A package for solving delay differential equations",
    long_description=open("README.md").read(),  # Read from README.md
    long_description_content_type="text/markdown",  # Specify Markdown format
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
