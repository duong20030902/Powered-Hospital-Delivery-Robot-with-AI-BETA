import React from "react";
import Header from "../components/Header";

const MainLayout = ({ children }) => {
    return (
        <div>
            <Header />

            <div className="container-fluid pt-5 mt-4">
                {children}
            </div>
        </div>
    );
};

export default MainLayout;
